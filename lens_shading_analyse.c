/*
Copyright (c) 2017, Raspberry Pi (Trading) Ltd
Copyright (c) 2017, Dave Stevenson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * \file lens_shading_analyse
 *
 * Description
 *
 * This application will take a Raw file captured using Raspistill
 * or similar, and analyse it in order to produce a customised lens shading
 * table. In order to get sensible results, the image should be of a
 * plain, uniformly illuminated scene (eg a nice white wall).
 *
 * It'll write out the four colour channels as ch1.bin-ch4.bin, viewable as
 * 16bit/pixel single channel images, although only the bottom 10 bits are used.
 * It also writes a file called ls_table.h, which provides the lens shading grid.
 * Pass that back to the camera component using code similar to:
    {
      MMAL_PARAMETER_LENS_SHADING_T ls = {{MMAL_PARAMETER_LENS_SHADING_OVERRIDE, sizeof(MMAL_PARAMETER_LENS_SHADING_T)}};
      void *grid;

      #include "ls_grid.h"

      ls.enabled = MMAL_TRUE;
      ls.grid_cell_size = 64;
      ls.grid_width = ls.grid_stride = grid_width;
      ls.grid_height = grid_height;
      ls.ref_transform = ref_transform;

      state->lens_shading = vcsm_malloc(ls.grid_stride*ls.grid_height*4, "ls_grid");
      ls.mem_handle_table = vcsm_vc_hdl_from_hdl(state->lens_shading);

      grid = vcsm_lock(state->lens_shading);

      memcpy(grid, ls_grid, vcos_min(sizeof(ls_grid), ls.grid_stride*ls.grid_height*4));

      vcsm_unlock_hdl(state->lens_shading);

      status = mmal_port_parameter_set(camera->control, &ls.hdr);
      if (status != MMAL_SUCCESS)
         vcos_log_error("Failed to set lens shading parameters - %d", status);
   }
 * and
      vcsm_free(state.lens_shading);
 * when finished (the firmware should acquire a reference on the vcsm allocation as it
 * is passed in, and only release it when done, so in theory you can release the handle
 * immediately after passing it in. Needs to be checked.)
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#define NUM_CHANNELS 4

//This structure is at offset 0xB0 from the 'BRCM' ident.
struct brcm_raw_header {
	uint8_t name[32];
	uint16_t width;
	uint16_t height;
	uint16_t padding_right;
	uint16_t padding_down;
	uint32_t dummy[6];
	uint16_t transform;
	uint16_t format;
	uint8_t bayer_order;
	uint8_t bayer_format;
};
//Values taken from https://github.com/raspberrypi/userland/blob/master/interface/vctypes/vc_image_types.h
#define BRCM_FORMAT_BAYER  33
#define BRCM_BAYER_RAW10   3

enum bayer_order_t {
	RGGB,
	GBRG,
	BGGR,
	GRBG
};

const int channel_ordering[4][4] = {
	{ 0, 1, 2, 3 },
	{ 2, 3, 0, 1 },
	{ 3, 2, 1, 0 },
	{ 1, 0, 3, 2 }
};

uint16_t black_level_correct(uint16_t raw_pixel, unsigned int black_level, unsigned int max_value)
{
	return ((raw_pixel - black_level) * max_value) / (max_value - black_level);
}

int main(int argc, char *argv[])
{
	int in = 0;
	FILE *out, *header, *table;
	int i, x, y;
	uint16_t *out_buf[NUM_CHANNELS];
	void *mmap_buf;
	uint8_t *in_buf;
	struct stat sb;
	int bayer_order;
	struct brcm_raw_header *hdr;
	int width, height, stride;
	int grid_width, grid_height;
	int single_channel_width, single_channel_height;
	unsigned int black_level = 16;

	if (argc < 2)
	{
		printf("%s <input filename> [black level]\n", argv[0]);
		return -1;
	}

	in = open(argv[1], O_RDONLY);
	if (in < 0)
	{
		printf("Failed to open %s\n", argv[1]);
		return -1;
	}
	if (argc >= 3)
	{
		black_level = strtoul(argv[2], NULL, 10);
	}

	fstat(in, &sb);
	printf("File size is %ld\n", sb.st_size);

	mmap_buf = mmap(NULL, sb.st_size, PROT_READ, MAP_PRIVATE, in, 0);
	if (mmap_buf == MAP_FAILED)
	{
		printf("mmap failed\n");
		goto close_file;
	}

	if (!memcmp(mmap_buf, "\xff\xd8", 2))
	{
		//JPEG+RAW - find the raw header
		//Try the appropriate offsets for the full res modes
		//of OV5647 and IMX219. Any other modes will need to be
		//stripped down to the bare raw (inc header) before processing
		in_buf = ((uint8_t*)mmap_buf) + sb.st_size - 6404096;
		if (memcmp(in_buf, "BRCM", 4))
		{
			//Failed on OV5647, try IMX219
			in_buf = ((uint8_t*)mmap_buf) + sb.st_size - 10270208;
			if (memcmp(in_buf, "BRCM", 4))
			{
				//Failed totally - reset to the start of the buffer
				in_buf = (uint8_t*)mmap_buf;
			}
		}
	}
	else
	{
		in_buf = (uint8_t*)mmap_buf;
	}

	if (strncmp(in_buf, "BRCM", 4))
	{
		printf("Raw file missing BRCM header\n");
		goto unmap;
	}

	hdr = (struct brcm_raw_header*) (in_buf+0xB0);
	printf("Header decoding: mode %s, width %u, height %u, padding %u %u\n",
			hdr->name, hdr->width, hdr->height, hdr->padding_right, hdr->padding_down);
	printf("transform %u, image format %u, bayer order %u, bayer format %u\n",
			hdr->transform, hdr->format, hdr->bayer_order, hdr->bayer_format);
	if (hdr->format != BRCM_FORMAT_BAYER || hdr->bayer_format != BRCM_BAYER_RAW10)
	{
		printf("Raw file is not Bayer raw10\n");
		goto unmap;
	}
	bayer_order = hdr->bayer_order;
	width = hdr->width;
	height = hdr->height;
	single_channel_width = width/2;
	single_channel_height = height/2;
	grid_width = single_channel_width / 32 + (single_channel_width % 32 == 0 ? 0 : 1);
	grid_height = single_channel_height / 32 + (single_channel_height % 32 == 0 ? 0 : 1);
	printf("Grid size: %d x %d\n", grid_width, grid_height);

	//Stride computed via same formula as the firmware uses.
	stride = (((((width + hdr->padding_right)*5)+3)>>2) + 31)&(~31);

	for (i=0; i<NUM_CHANNELS; i++)
	{
		out_buf[i] = (uint16_t*)malloc(single_channel_width*single_channel_height * sizeof(uint16_t));
		memset(out_buf[i], 0, single_channel_width*single_channel_height * sizeof(uint16_t));
	}

	for (y=0; y<height; y++)
	{
		uint8_t *line = in_buf + (y*stride) + 32768;
		int chan_a, chan_b;
		if (y&1)
		{
			chan_a = 2;
			chan_b = 3;
		}
		else
		{
			chan_a = 0;
			chan_b = 1;
		}

		uint16_t *chan_a_line = out_buf[chan_a] + ((y>>1)*single_channel_width);
		uint16_t *chan_b_line = out_buf[chan_b] + ((y>>1)*single_channel_width);
		for (x=0; x<width; x+=4)
		{
			uint8_t lsbs = line[4];
			*(chan_a_line) = black_level_correct(((*line)<<2) + (lsbs>>6), black_level, (1<<10)-1);
			chan_a_line++;
			lsbs<<=2;
			line++;
			*(chan_b_line) = black_level_correct(((*line)<<2) + (lsbs>>6), black_level, (1<<10)-1);
			chan_b_line++;
			lsbs<<=2;
			line++;
			*(chan_a_line) = black_level_correct(((*line)<<2) + (lsbs>>6), black_level, (1<<10)-1);
			chan_a_line++;
			lsbs<<=2;
			line++;
			*(chan_b_line) = black_level_correct(((*line)<<2) + (lsbs>>6), black_level, (1<<10)-1);
			chan_b_line++;
			lsbs<<=2;
			line++;
			line++; //skip the LSBs
		}
	}

	printf("Save data. Bayer order is %d\n", bayer_order);

	header = fopen("ls_table.h", "wb");
	table = fopen("ls_table.txt", "wb");
	fprintf(header, "uint8_t ls_grid[] = {\n");
	for (i=0; i<NUM_CHANNELS; i++)
	{
		// Write out the raw data for analysis
		const char *filenames[NUM_CHANNELS] = {
			"ch1.bin",
			"ch2.bin",
			"ch3.bin",
			"ch4.bin"
		};
		out = fopen(filenames[i], "wb");
		if (out)
		{
			printf("Saving %s data\n", filenames[i]);
			fwrite(out_buf[i], (single_channel_width*single_channel_height)*sizeof(uint16_t), 1, out);
			fclose(out);
			out = NULL;
		}

		//Write out the lens shading table in the order RGGB
		uint16_t *channel = out_buf[channel_ordering[bayer_order][i]];
		int mid_value_avg = 0;
		int count = 0;
		//Average out the centre 64 pixels.
		for (x=(single_channel_width>>1)-4; x<=(single_channel_width>>1)+4; x++)
		{
			for (y=(single_channel_height>>1)-4; y<=(single_channel_height>>1)+4; y++)
			{
				mid_value_avg += channel[x + y*single_channel_width];
				count++;
			}
		}
		uint16_t middle_val = (mid_value_avg / count) << 5;

		const char *channel_comments[4] = {
			"R",
			"Gr",
			"Gb",
			"B"
		};
		printf("Middle_val is %d\n", middle_val);

		fprintf(header, "//%s - Ch %d\n", channel_comments[i], channel_ordering[bayer_order][i]);
		uint16_t *line;
		for (y=16; y<single_channel_height+32; y+=32)	//Grid size is 64x64, but the component tables are subsampled due to the Bayer pattern
		{
			if (y>=single_channel_height)
				line = &channel[(single_channel_height-1)*(single_channel_width)];
			else
				line = &channel[y*(single_channel_width)];

			for(x=16; x<single_channel_width; x+=32)
			{
				//Average 3 pixels horizontally to give some noise rejection
				int avg = line[x] + line[x-1] + line[x+1];
				int gain = (middle_val*3) / avg;
				if (gain > 255)
					gain = 255;	//Clip as uint8_t
				else if (gain < 32)
					gain = 32;	//Clip at x1.0
				fprintf(header, "%d, ", gain );
				fprintf(table, "%d %d %d %d\n", x, y, gain, i );
			}
			//Compute edge value from the very edge 2 pixels.
			{
				int avg = line[single_channel_width-2] + line[single_channel_width-1];
				int gain = (middle_val*2) / avg;
				if (gain > 255)
					gain = 255;	//Clip as uint8_t
				else if (gain < 32)
					gain = 32;	//Clip at x1.0
				fprintf(header, "%d,\n", gain );
				fprintf(table, "%d %d %d %d\n", x, y, gain, i );
			}
		}

	}
	fprintf(header, "};\n");
	fprintf(header, "uint32_t ref_transform = %u;\n", hdr->transform);
	fprintf(header, "uint32_t grid_width = %u;\n", grid_width);
	fprintf(header, "uint32_t grid_height = %u;\n", grid_height);

	for (i=0; i<NUM_CHANNELS; i++)
	{
		 free(out_buf[i]);
	}
unmap:
	munmap(mmap_buf, sb.st_size);
close_file:
	close(in);
	return 0;
}

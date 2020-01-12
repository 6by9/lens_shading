This app takes a raw or JPEG+raw file created on a Pi and analyses it to create a lens shading table.

The image should be of a well illuminated uniform scene (eg plain white sheet of paper or wall),
as it does a simple comparison of values at the appropriate points to make up a compensation table.

It'll write out the four colour channels as ch1.bin-ch4.bin, viewable as
16bit/pixel single channel images, although only the bottom 10 bits are used.
It also writes a file called ls_table.h, which provides the lens shading grid.
Pass that back to the camera component using code similar to:
```
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
```
and
```
      vcsm_free(state.lens_shading);
```
when finished.

ls_table.txt is a comma separated file for easy visualization with Gnuplot. For a colored plot of all samples:
```
set palette defined (0 "red", 1 "yellow", 2 "magenta", 3 "blue")
splot "ls_table.txt" using 1:2:3:4 w p ps 0.75 pt 7 lc palette z notitle
```
Single sample plot ($4==0 => red):
```
splot "ls_table.txt" using 1:2:($4==0?$3:1/0)
```

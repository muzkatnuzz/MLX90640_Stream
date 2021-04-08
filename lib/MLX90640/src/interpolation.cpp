#include <Arduino.h>

// write array of rows and cols to serial out
// break per row
void print_array(float *src, uint16_t src_rows, uint16_t src_cols)
{
  for (size_t i = 0; i < (src_rows * src_cols); i++)
  {
    if (i % src_cols == 0)
    {
      Serial.println();
    } 

    Serial.printf("%.2f, ", src[i]);

    vTaskDelay(1);
  }
}

// src is a grid src_rows * src_cols
// dest is a pre-allocated grid, dest_rows*dest_cols
// source: http://tech-algorithm.com/articles/nearest-neighbor-image-scaling
void interpolate_image_nearest_neighbour(float *src, uint16_t src_width, uint16_t src_heigth, float *dest, uint16_t dest_width, uint16_t dest_height)
{
  // added +1 to account for an early rounding problem
  uint16_t x_ratio = (uint16_t)((src_width << 16) / dest_width) + 1; //<<16 is equivalent to x * 65536
  uint16_t y_ratio = (uint16_t)((src_heigth << 16) / dest_height) + 1; //<<16 is equivalent to x * 65536

  //log_d("interpolate_image_nearest_neighbour Scale: \t x=%u, y=%u", x_ratio, y_ratio);
  
  // log_d("Original array (%u x %u)", src_width, src_heigth);
  // print_array(src, src_heigth, src_width);

  uint16_t x2, y2;
  for (int col = 0; col < dest_height; col++)
  {
    for (int row = 0; row < dest_width; row++)
    {
      x2 = ((row * x_ratio) >> 16); //>>16 is equivalent to divide by 65536
      y2 = ((col * y_ratio) >> 16); //>>16 is equivalent to divide by 65536
      dest[(col * dest_width) + row] = src[(y2 * src_width) + x2];
    }
  }
  
  // log_d("Interpolated array (%u x %u)", dest_width, dest_height);
  // print_array(dest, dest_height, dest_width);
}

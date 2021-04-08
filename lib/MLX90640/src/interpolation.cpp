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
void interpolate_image_nearest_neighbour(float *src, uint16_t src_rows, uint16_t src_cols, float *dest, uint16_t dest_rows, uint16_t dest_cols)
{
  // added +1 to account for an early rounding problem
  uint16_t x_ratio = (uint16_t)((src_cols << 16) / dest_cols) + 1; //<<16 is equivalent to x * 65536
  uint16_t y_ratio = (uint16_t)((src_rows << 16) / dest_rows) + 1; //<<16 is equivalent to x * 65536

  //log_d("interpolate_image_nearest_neighbour Scale: \t x=%u, y=%u", x_ratio, y_ratio);
  
  // log_d("Original array (%u x %u)", src_cols, src_rows);
  // print_array(src, src_rows, src_cols);

  uint16_t x2, y2;
  for (int col = 0; col < dest_rows; col++)
  {
    for (int row = 0; row < dest_cols; row++)
    {
      x2 = ((row * x_ratio) >> 16); //>>16 is equivalent to divide by 65536
      y2 = ((col * y_ratio) >> 16); //>>16 is equivalent to divide by 65536
      dest[(col * dest_cols) + row] = src[(y2 * src_cols) + x2];
    }
  }
  
  // log_d("Interpolated array (%u x %u)", dest_cols, dest_rows);
  // print_array(dest, dest_rows, dest_cols);
}

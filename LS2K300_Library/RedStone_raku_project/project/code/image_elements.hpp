#ifndef _IMAGE_ELEMENTS_HPP
#define _IMAGE_ELEMENTS_HPP

void cross_fill(uint8(*image)[IMAGE_W], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
										 uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2]);
                                         
#endif
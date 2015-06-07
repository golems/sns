#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sns.h>
#include <sns/msg.h>


#ifdef _cplusplus
extern "C" {
#endif

struct sns_msg_sdh_tactile {

  struct sns_msg_header header;
  float cog_x[6]; /**< CoG x coordinate in each of the 6 SDH pads */
  float cog_y[6];  /**< CoG y coordinate in each of the 6 SDH pads */
  float area[6]; /**< Contact area per pad */
  float force[6]; /**< Force applied in each pad */
  uint16_t x[1]; // Tactile info per each independent cell

};

SNS_DEF_MSG_VAR( sns_msg_sdh_tactile,  x );
SNS_DEC_MSG_PLUGINS( sns_msg_sdh_tactile );



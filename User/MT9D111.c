#include "MT9D111.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "camera_hw.h"
#include "def.h"
#include "usbd_cdc_vcp.h"
#include "systick.h"
#include "i2c.h"

#define RET_OK                  0
#define RET_ERROR               -1
#define SENSOR_PAGE_REG         0xF0
#define CAM_I2C_SLAVE_ADDR      0xBA//((0xBA >> 1))

typedef struct MT9D111RegLst
{
    unsigned char ucPageAddr;
    unsigned char ucRegAddr;
    unsigned short usValue;
} s_RegList;
#ifndef ENABLE_JPEG
static const s_RegList preview_on_cmd_list[]= {
    {1, 0xC6, 0xA103    },  // SEQ_CMD
    {1, 0xC8, 0x0001    },  // SEQ_CMD, Do Preview
    {1, 0xC6, 0xA104    },  // SEQ_CMD
    {111, 0xC8, 0x0003  },  // SEQ_CMD, Do Preview
    {1, 0xC6, 0xA103    },  // SEQ_CMD-refresh
    {1, 0xC8, 0x0005    },  // SEQ_CMD-refresh
    {1, 0xC6, 0xA103    },  // SEQ_CMD-refresh
    {1, 0xC8, 0x0006    },  // SEQ_CMD-refresh
    {1, 0xC6, 0xA104    },  // SEQ_CMD
    {111, 0xC8, 0x0003  },  // SEQ_CMD, Do Preview
    {100, 0x00, 0x01E0  },  // Delay = 500ms
};

static  const s_RegList freq_setup_cmd_List[]= {
    {1, 0xC6, 0x276D    },  // MODE_FIFO_CONF1_A
    {1, 0xC8, 0xE4E2    },  // MODE_FIFO_CONF1_A
    {1, 0xC6, 0xA76F    },  // MODE_FIFO_CONF2_A
    {1, 0xC8, 0x00E8    },  // MODE_FIFO_CONF2_A
    {1, 0xC6, 0xA103    },  // SEQ_CMD
    {1, 0xC8, 0x0005    },  // SEQ_CMD (Refresh)
   // Set maximum integration time to get a minimum of 15 fps at 45MHz
    {1, 0xC6, 0xA20E    },  // AE_MAX_INDEX
    {1, 0xC8, 0x0004},      // AE_MAX_INDEX
    {1, 0xC6, 0xA102    },  // SEQ_MODE
    {1, 0xC8, 0x0001    },  // SEQ_MODE
    {1, 0xC6, 0xA102    },  // SEQ_MODE
    {1, 0xC8, 0x0005    },  // SEQ_MODE
   // Set minimum integration time to get a maximum of 15 fps at 45MHz
    {1, 0xC6, 0xA20D    },  // AE_MAX_INDEX
    {1, 0xC8, 0x0004    },  // AE_MAX_INDEX
    {1, 0xC6, 0xA103    },  // SEQ_CMD
    {1, 0xC8, 0x0005    },  // SEQ_CMD (Refresh)
};

static  const s_RegList image_size_240_320_preview_cmds_list[]=
{
    {0, 0x07, 0x00FE    },  // HORZ_BLANK_A
    {0, 0x08, 0x02A0    },  // VERT_BLANK_A
    {0, 0x20, 0x0303    },  // READ_MODE_B (Image flip settings)
    {0, 0x21, 0x8400    },  // READ_MODE_A (1ADC)
    {1, 0xC6, 0x2703    },  // MODE_OUTPUT_WIDTH_A
//    {1, 0xC8, 0x00F0    },  // MODE_OUTPUT_WIDTH_A
    {1, 0xC8, 0x00A0    },  // MODE_OUTPUT_WIDTH_A:160		
    {1, 0xC6, 0x2705    },  // MODE_OUTPUT_HEIGHT_A
//    {1, 0xC8, 0x0140    },  // MODE_OUTPUT_HEIGHT_A
		{1, 0xC8, 0x078    },  // MODE_OUTPUT_HEIGHT_A:120		
    {1, 0xC6, 0x2727    },  // MODE_CROP_X0_A
    {1, 0xC8, 0x0000    },  // MODE_CROP_X0_A
    {1, 0xC6, 0x2729    },  // MODE_CROP_X1_A
    {1, 0xC8, 0x00F0    },  // MODE_CROP_X1_A:240
    {1, 0xC6, 0x272B    },  // MODE_CROP_Y0_A
    {1, 0xC8, 0x0000    },  // MODE_CROP_Y0_A
    {1, 0xC6, 0x272D    },  // MODE_CROP_Y1_A
    {1, 0xC8, 0x0140    },  // MODE_CROP_Y1_A:320
    {1, 0xC6, 0x270F    },  // MODE_SENSOR_ROW_START_A
    {1, 0xC8, 0x001C    },  // MODE_SENSOR_ROW_START_A
    {1, 0xC6, 0x2711    },  // MODE_SENSOR_COL_START_A
    {1, 0xC8, 0x003C    },  // MODE_SENSOR_COL_START_A
    {1, 0xC6, 0x2713    },  // MODE_SENSOR_ROW_HEIGHT_A
    {1, 0xC8, 0x0280    },  // MODE_SENSOR_ROW_HEIGHT_A:640
    {1, 0xC6, 0x2715    },  // MODE_SENSOR_COL_WIDTH_A
    {1, 0xC8, 0x03C0    },  // MODE_SENSOR_COL_WIDTH_A:960
    {1, 0xC6, 0x2717    },  // MODE_SENSOR_X_DELAY_A
    {1, 0xC8, 0x0088    },  // MODE_SENSOR_X_DELAY_A
    {1, 0xC6, 0x2719    },  // MODE_SENSOR_ROW_SPEED_A
    {1, 0xC8, 0x0011    },  // MODE_SENSOR_ROW_SPEED_A
    {1, 0xC6, 0xA103    },  // SEQ_CMD
    {1, 0xC8, 0x0005    },  // SEQ_CMD
    {1, 0xC6, 0xA103    },  // SEQ_CMD
    {1, 0xC8, 0x0006    },  // SEQ_CMD
};

static  const s_RegList preview_cmds_list[]= {

    {1, 0xC6, 0xA77D    },  // MODE_OUTPUT_FORMAT_A
    {1, 0xC8, 0x0020    },  // MODE_OUTPUT_FORMAT_A; RGB565
    {1, 0xC6, 0x270B    },  // MODE_CONFIG
    {1, 0xC8, 0x0030    },  // MODE_CONFIG, JPEG disabled for A and B
    {1, 0xC6, 0xA103    },  // SEQ_CMD
    {1, 0xC8, 0x0005    }   // SEQ_CMD, refresh
};
#else 
static  const s_RegList capture_cmds_list[]= {
    {0, 0x65, 0xA000    },  // Disable PLL
    {0, 0x65, 0xE000    },  // Power DOWN PLL
    {100, 0x00, 0x01F4  },  // Delay =500ms
    {0,  0x66,  0x500B  },
    {0,  0x67,  0x0500  },
    {0, 0x65,   0xA000  },  // Disable PLL
    {0,  0x65,  0x2000  },  // Enable PLL
    {0, 0x20, 0x0000    },  // READ_MODE_B (Image flip settings)
    {100, 0x00, 0x01F4  },  // Delay =500ms
    {100, 0x00, 0x01F4  },  // Delay =500ms
    {100, 0x00, 0x01F4  },  // Delay =500ms
    {1, 0xC6, 0xA102    },  // SEQ_MODE
    {1, 0xC8, 0x0001    },  // SEQ_MODE
    {1, 0xC6, 0xA102    },  // SEQ_MODE
    {1, 0xC8, 0x0005    },  // SEQ_MODE
    {1,  0xC6, 0xA120   },  // Enable Capture video
    {1,  0xC8, 0x0002   },
    {1,  0xC6, 0x270B   },  // Mode config, disable JPEG bypass
    {1,  0xC8, 0x0000   },
    {1,  0xC6, 0x2702   },  // FIFO_config0b, no spoof, adaptive clock
    {1,  0xC8, 0x001E   },
    {1,  0xC6, 0xA907   },  // JPEG mode config, video
    {1,  0xC8, 0x0035   },
    {1,  0xC6, 0xA906   },  // Format YCbCr422
    {1,  0xC8, 0x0000   },
    {1,  0xC6, 0xA90A   },  // Set the qscale1
    {1,  0xC8, 0x0089   },
    {1,  0xC6, 0x2908   },  // Set the restartInt
    {1,  0xC8, 0x0020   },
};

static s_RegList start_jpeg_capture_cmd_list[]={
    {1, 0xC6, 0xA103    },  // SEQ_CMD, Do capture
    {1, 0xC8, 0x0002    },
    {100, 0x00, 0x01F4  },  // Delay =500ms
};

static s_RegList stop_jpeg_capture_cmd_list[]={
    {1, 0xC6, 0xA103    },  // SEQ_CMD, Do capture
    {1, 0xC8, 0x0001    },
    {100, 0x00, 0x01F4  },  // Delay =500ms
};

#define INDEX_CROP_X0           1
#define INDEX_CROP_X1           3
#define INDEX_CROP_Y0           5
#define INDEX_CROP_Y1           7
#define INDEX_SIZE_WIDTH        12//9
#define INDEX_SIZE_HEIGHT       14//11
static  s_RegList resolution_cmds_list[]= {
    {100, 0x00, 0x01F4      }, // Delay =500ms
    {1, 0xC6, 0x2735        }, //MODE_CROP_X0_A
    {1, 0xC8, 0x0000        }, //MODE_CROP_X0_A
    {1, 0xC6, 0x2737        }, //MODE_CROP_X1_A
    {1, 0xC8, 1600          }, //MODE_CROP_X1_A
    {1, 0xC6, 0x2739        }, //MODE_CROP_Y0_A
    {1, 0xC8, 0x0000        }, //MODE_CROP_Y0_A
    {1, 0xC6, 0x273B        }, //MODE_CROP_Y1_A
    {1, 0xC8, 1200          }, //MODE_CROP_Y1_A   
    {1, 0xC6, 0xA103        }, // SEQ_CMD, Do capture  
    {1, 0xC8, 0x0005        },
    
    {1, 0xC6, 0x2707        }, //MODE_OUTPUT_WIDTH_B
    {1, 0xC8, 640           }, //MODE_OUTPUT_WIDTH_B
    {1, 0xC6, 0x2709        }, //MODE_OUTPUT_HEIGHT_B
    {1, 0xC8, 480           }, //MODE_OUTPUT_HEIGHT_B   
};
#endif 

static const s_RegList init_cmds_list[]= {
    {100,0x00,0x01F4},
		{0, 0x66, 0x2401},
		{0, 0x67, 0x501},
    {0, 0x33, 0x0343    }, // RESERVED_CORE_33
    {1, 0xC6, 0xA115    }, // SEQ_LLMODE
    {1, 0xC8, 0x0020    }, // SEQ_LLMODE
    {0, 0x38, 0x0866    }, // RESERVED_CORE_38
    {2, 0x80, 0x0168    }, // LENS_CORRECTION_CONTROL
    {2, 0x81, 0x6432    }, // ZONE_BOUNDS_X1_X2
    {2, 0x82, 0x3296    }, // ZONE_BOUNDS_X0_X3
    {2, 0x83, 0x9664    }, // ZONE_BOUNDS_X4_X5
    {2, 0x84, 0x5028    }, // ZONE_BOUNDS_Y1_Y2
    {2, 0x85, 0x2878    }, // ZONE_BOUNDS_Y0_Y3
    {2, 0x86, 0x7850    }, // ZONE_BOUNDS_Y4_Y5
    {2, 0x87, 0x0000    }, // CENTER_OFFSET
    {2, 0x88, 0x0152    }, // FX_RED
    {2, 0x89, 0x015C    }, // FX_GREEN
    {2, 0x8A, 0x00F4    }, // FX_BLUE
    {2, 0x8B, 0x0108    }, // FY_RED
    {2, 0x8C, 0x00FA    }, // FY_GREEN
    {2, 0x8D, 0x00CF    }, // FY_BLUE
    {2, 0x8E, 0x09AD    }, // DF_DX_RED
    {2, 0x8F, 0x091E    }, // DF_DX_GREEN
    {2, 0x90, 0x0B3F    }, // DF_DX_BLUE
    {2, 0x91, 0x0C85    }, // DF_DY_RED
    {2, 0x92, 0x0CFF    }, // DF_DY_GREEN
    {2, 0x93, 0x0D86    }, // DF_DY_BLUE
    {2, 0x94, 0x163A    }, // SECOND_DERIV_ZONE_0_RED
    {2, 0x95, 0x0E47    }, // SECOND_DERIV_ZONE_0_GREEN
    {2, 0x96, 0x103C    }, // SECOND_DERIV_ZONE_0_BLUE
    {2, 0x97, 0x1D35    }, // SECOND_DERIV_ZONE_1_RED
    {2, 0x98, 0x173E    }, // SECOND_DERIV_ZONE_1_GREEN
    {2, 0x99, 0x1119    }, // SECOND_DERIV_ZONE_1_BLUE
    {2, 0x9A, 0x1663    }, // SECOND_DERIV_ZONE_2_RED
    {2, 0x9B, 0x1569    }, // SECOND_DERIV_ZONE_2_GREEN
    {2, 0x9C, 0x104C    }, // SECOND_DERIV_ZONE_2_BLUE
    {2, 0x9D, 0x1015    }, // SECOND_DERIV_ZONE_3_RED
    {2, 0x9E, 0x1010    }, // SECOND_DERIV_ZONE_3_GREEN
    {2, 0x9F, 0x0B0A    }, // SECOND_DERIV_ZONE_3_BLUE
    {2, 0xA0, 0x0D53    }, // SECOND_DERIV_ZONE_4_RED
    {2, 0xA1, 0x0D51    }, // SECOND_DERIV_ZONE_4_GREEN
    {2, 0xA2, 0x0A44    }, // SECOND_DERIV_ZONE_4_BLUE
    {2, 0xA3, 0x1545    }, // SECOND_DERIV_ZONE_5_RED
    {2, 0xA4, 0x1643    }, // SECOND_DERIV_ZONE_5_GREEN
    {2, 0xA5, 0x1231    }, // SECOND_DERIV_ZONE_5_BLUE
    {2, 0xA6, 0x0047    }, // SECOND_DERIV_ZONE_6_RED
    {2, 0xA7, 0x035C    }, // SECOND_DERIV_ZONE_6_GREEN
    {2, 0xA8, 0xFE30    }, // SECOND_DERIV_ZONE_6_BLUE
    {2, 0xA9, 0x4625    }, // SECOND_DERIV_ZONE_7_RED
    {2, 0xAA, 0x47F3    }, // SECOND_DERIV_ZONE_7_GREEN
    {2, 0xAB, 0x5859    }, // SECOND_DERIV_ZONE_7_BLUE
    {2, 0xAC, 0x0000    }, // X2_FACTORS
    {2, 0xAD, 0x0000    }, // GLOBAL_OFFSET_FXY_FUNCTION
    {2, 0xAE, 0x0000    }, // K_FACTOR_IN_K_FX_FY
    {1, 0x08, 0x01FC    }, // COLOR_PIPELINE_CONTROL
    {1, 0xC6, 0x2003    }, // MON_ARG1
    {1, 0xC8, 0x0748    }, // MON_ARG1
    {1, 0xC6, 0xA002    }, // MON_CMD
    {1, 0xC8, 0x0001    }, // MON_CMD
    {111, 0xC8,0x0000 },
    {1, 0xC6, 0xA361    }, // AWB_TG_MIN0
    {1, 0xC8, 0x00E2    }, // AWB_TG_MIN0
    {1, 0x1F, 0x0018    }, // RESERVED_SOC1_1F
    {1, 0x51, 0x7F40    }, // RESERVED_SOC1_51
    {0, 0x33, 0x0343    }, // RESERVED_CORE_33
    {0, 0x38, 0x0868    }, // RESERVED_CORE_38
    {1, 0xC6, 0xA10F    }, // SEQ_RESET_LEVEL_TH
    {1, 0xC8, 0x0042    }, // SEQ_RESET_LEVEL_TH
    {1, 0x1F, 0x0020    }, // RESERVED_SOC1_1F
    {1, 0xC6, 0xAB04    }, // HG_MAX_DLEVEL
    {1, 0xC8, 0x0008    }, // HG_MAX_DLEVEL
    {1, 0xC6, 0xA103    }, // SEQ_CMD
    {1, 0xC8, 0x0005    }, // SEQ_CMD
    {1, 0xC6, 0xA104    }, // SEQ_CMD
    {111, 0xC8,0x0003   },
    {1, 0x08, 0x01FC    }, // COLOR_PIPELINE_CONTROL
    {1, 0x08, 0x01EC    }, // COLOR_PIPELINE_CONTROL
    {1, 0x08, 0x01FC    }, // COLOR_PIPELINE_CONTROL
    {1, 0x36, 0x0F08    }, // APERTURE_PARAMETERS
    {1, 0xC6, 0xA103    }, // SEQ_CMD
    {1, 0xC8, 0x0005    }, // SEQ_CMD
};

extern volatile uint32_t Cam_Capture[];
long lRetVal = -1;
void MT9D111_HW_DCMI_Init(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;
	DCMI_InitTypeDef DCMI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DCMI_CROPInitTypeDef CROP_InitStructure;

  /*** Configures the DCMI GPIOs to interface with the MT9V034 camera module ***/
  /* Enable DCMI GPIOs clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC |
                         RCC_AHB1Periph_GPIOE, ENABLE);
	//Enable the clock for the DCMI
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);
	
  /* Connect DCMI pins to AF13 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI);
	
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_DCMI);
  //GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_DCMI);

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_DCMI);
	
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI);
  
  /* DCMI GPIO configuration */
  /* HREF,PCLK(PA4/6) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* D5(PB6), VSYNC(PB7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* D0..D3, D8, D9(PC6..PC9, PC10, PC12) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_11;// | GPIO_Pin_10 | GPIO_Pin_12;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	  /* D6,D7(PE5..PE6) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	DCMI_DeInit();

	// DCMI configuration
	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_SnapShot;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;		
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_High;
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
	
	/*
	Data Frame: R G R G R G ......
							G B G B G B ......
							R G R G R G ......
	*/
	
	//240*180
	CROP_InitStructure.DCMI_VerticalStartLine = 0;
	CROP_InitStructure.DCMI_HorizontalOffsetCount = 0;	//250
	CROP_InitStructure.DCMI_VerticalLineCount = FULL_IMAGE_ROW_SIZE - 1;
	CROP_InitStructure.DCMI_CaptureCount = BYTES_PER_PIXEL*FULL_IMAGE_COLUMN_SIZE - 1;//2 byte RGB 565
	
	DCMI_CROPConfig(&CROP_InitStructure);
	DCMI_CROPCmd(ENABLE);

	DCMI_Init(&DCMI_InitStructure);

	//Enable DCMI Interrupt
	DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
//	DCMI_ITConfig(DCMI_IT_FRAME | DCMI_IT_LINE, ENABLE);
	DCMI_Cmd(ENABLE);
	
  /* Enable the DCMI Stream IRQ Channel */
  NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   
}


//*****************************************************************************
//
//! This function implements the Register Write in MT9D111 sensor
//!
//! \param1                     Register List
//! \param2                     No. Of Items
//!
//! \return                     0 - Success
//!                             -1 - Error
//
//*****************************************************************************
static long RegLstWrite(s_RegList *pRegLst, unsigned long ulNofItems)
{
    unsigned long       ulNdx;
    unsigned short      usTemp;
    unsigned char       i;
    unsigned char       ucBuffer[20];
    unsigned long       ulSize;
    long lRetVal = -1;

    if(pRegLst == NULL)
    {
        return RET_ERROR;
    }
    
    for(ulNdx = 0; ulNdx < ulNofItems; ulNdx++)
    {
        if(pRegLst->ucPageAddr == 100)
        {
            // PageAddr == 100, insret a delay equal to reg value 
            SYSTIM_DelayTms(pRegLst->usValue);
        }
        else if(pRegLst->ucPageAddr == 111)
        {
            // PageAddr == 111, wait for specified register value 
            do
            {
                ucBuffer[0] = pRegLst->ucRegAddr;
								I2C_WriteBytes(I2C2, 0, CAM_I2C_SLAVE_ADDR, ucBuffer[0], 0);
								//lRetVal = I2CBufferWrite(CAM_I2C_SLAVE_ADDR,ucBuffer,1,1);
                //ASSERT_ON_ERROR(lRetVal);
                if (!I2C_ReadBytes_NoReStart(I2C2, ucBuffer, CAM_I2C_SLAVE_ADDR, 2))
								//if(I2CBufferRead(CAM_I2C_SLAVE_ADDR,ucBuffer,2,1))
                {
                    return RET_ERROR;
                }

                usTemp = ucBuffer[0] << 8;
                usTemp |= ucBuffer[1];
            }while(usTemp != pRegLst->usValue);
        }
        else
        {
            // Set the page 
            ucBuffer[0] = SENSOR_PAGE_REG;
            ucBuffer[1] = 0x00;
            ucBuffer[2] = (unsigned char)(pRegLst->ucPageAddr);
						if (!I2C_WriteBytes(I2C2, ucBuffer+1, CAM_I2C_SLAVE_ADDR, ucBuffer[0], 2))            
						//if(0 != I2CBufferWrite(CAM_I2C_SLAVE_ADDR,ucBuffer,3,I2C_SEND_STOP))
            {
                return RET_ERROR;
            }

            ucBuffer[0] = SENSOR_PAGE_REG;
						I2C_WriteBytes(I2C2, 0, CAM_I2C_SLAVE_ADDR, ucBuffer[0], 0);						
            //lRetVal = I2CBufferWrite(CAM_I2C_SLAVE_ADDR,ucBuffer,1,I2C_SEND_STOP);
            //ASSERT_ON_ERROR(lRetVal);
            I2C_ReadBytes_NoReStart(I2C2, ucBuffer, CAM_I2C_SLAVE_ADDR, 2);						
            //lRetVal = I2CBufferRead(CAM_I2C_SLAVE_ADDR,ucBuffer,2,I2C_SEND_STOP);
            //ASSERT_ON_ERROR(lRetVal);

            ucBuffer[0] = pRegLst->ucRegAddr;

            if(pRegLst->ucPageAddr  == 0x1 && pRegLst->ucRegAddr == 0xC8)
            {
                usTemp = 0xC8;
                i=1;
                while(pRegLst->ucRegAddr == usTemp)
                {
                    ucBuffer[i] = (unsigned char)(pRegLst->usValue >> 8);
                    ucBuffer[i+1] = (unsigned char)(pRegLst->usValue & 0xFF);
                    i += 2;
                    usTemp++;
                    pRegLst++;
                    ulNdx++;
                }

                ulSize = (i-2)*2 + 1;
                ulNdx--;
                pRegLst--;
            }
            else
            {
                ulSize = 3;
                ucBuffer[1] = (unsigned char)(pRegLst->usValue >> 8);
                ucBuffer[2] = (unsigned char)(pRegLst->usValue & 0xFF);
            }
						if (!I2C_WriteBytes(I2C2, ucBuffer+1, CAM_I2C_SLAVE_ADDR, ucBuffer[0], ulSize-1))
            //if(0 != I2CBufferWrite(CAM_I2C_SLAVE_ADDR,ucBuffer,
            //                                          ulSize,I2C_SEND_STOP))
            {
                return RET_ERROR;
            }

						//I2C_WriteBytes(I2C2, 0, CAM_I2C_SLAVE_ADDR, ucBuffer[0], 0);						
						//lRetVal = I2CBufferWrite(CAM_I2C_SLAVE_ADDR,ucBuffer,1,I2C_SEND_STOP);
						//ASSERT_ON_ERROR(lRetVal);
						//I2C_ReadBytes_NoReStart(I2C2, ucBuffer+1, CAM_I2C_SLAVE_ADDR, 2);
        }

        pRegLst++;
        SYSTIM_DelayTms(10);
    }

    return RET_OK;
}

void MT9D111_Init(void)
{
	uint8_t data[10]={0};

	Camera_HW_I2C_Init();
	SYSTIM_DelayTms(100);
	//CONFIG
	lRetVal = RegLstWrite((s_RegList *)init_cmds_list, \
                                    sizeof(init_cmds_list)/sizeof(s_RegList));	
	lRetVal = RegLstWrite((s_RegList *)preview_cmds_list,
										sizeof(preview_cmds_list)/sizeof(s_RegList));
	lRetVal = RegLstWrite((s_RegList *)image_size_240_320_preview_cmds_list, \
									sizeof(image_size_240_320_preview_cmds_list)/ \
									sizeof(s_RegList));
	lRetVal = RegLstWrite((s_RegList *)freq_setup_cmd_List,
									sizeof(freq_setup_cmd_List)/sizeof(s_RegList));
	lRetVal = RegLstWrite((s_RegList *)preview_on_cmd_list,
									sizeof(preview_on_cmd_list)/sizeof(s_RegList));
	SYSTIM_DelayTms(100);	
	MT9D111_HW_DCMI_Init();
	DMA_DCMI_Init((uint32_t)&Cam_Capture[0],FULL_IMAGE_SIZE>>2);
	
//	// Set the page 
//	data[0] = SENSOR_PAGE_REG;
//	data[1] = 0x00;
//	data[2] = 0x0;
//	I2C_WriteBytes(I2C2, data+1, CAM_I2C_SLAVE_ADDR, data[0], 2);            
//	//if(0 != I2CBufferWrite(CAM_I2C_SLAVE_ADDR,ucBuffer,3,I2C_SEND_STOP)
//	I2C_WriteBytes(I2C2, 0, CAM_I2C_SLAVE_ADDR, 3, 0);						
//	//lRetVal = I2CBufferWrite(CAM_I2C_SLAVE_ADDR,ucBuffer,1,I2C_SEND_STOP);
//	//ASSERT_ON_ERROR(lRetVal);
//	I2C_ReadBytes_NoReStart(I2C2, data+1, CAM_I2C_SLAVE_ADDR, 2);
//	data[3] = 3;
}

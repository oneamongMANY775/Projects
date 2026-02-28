/*!
  General BMS Library
@verbatim

@endverbatim
REVISION HISTORY
$Revision: 7139 $
$Date: 2017-4

Copyright (c) 2017, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.

Copyright 2017 Linear Technology Corp. (LTC)
***********************************************************/
#include "LTC681x.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"



const uint16_t crc15Table[256] = {
0x0000, 0xC599, 0xCEAB, 0x0B32, 0xD8CF, 0x1D56, 0x1664, 0xD3FD,
0xF407, 0x319E, 0x3AAC, 0xFF35, 0x2CC8, 0xE951, 0xE263, 0x27FA,
0xAD97, 0x680E, 0x633C, 0xA6A5, 0x7558, 0xB0C1, 0xBBF3, 0x7E6A,
0x5990, 0x9C09, 0x973B, 0x52A2, 0x815F, 0x44C6, 0x4FF4, 0x8A6D,
0x5B2E, 0x9EB7, 0x9585, 0x501C, 0x83E1, 0x4678, 0x4D4A, 0x88D3,
0xAF29, 0x6AB0, 0x6182, 0xA41B, 0x77E6, 0xB27F, 0xB94D, 0x7CD4,
0xF6B9, 0x3320, 0x3812, 0xFD8B, 0x2E76, 0xEBEF, 0xE0DD, 0x2544,
0x02BE, 0xC727, 0xCC15, 0x098C, 0xDA71, 0x1FE8, 0x14DA, 0xD143,
0xF3C5, 0x365C, 0x3D6E, 0xF8F7, 0x2B0A, 0xEE93, 0xE5A1, 0x2038,
0x07C2, 0xC25B, 0xC969, 0x0CF0, 0xDF0D, 0x1A94, 0x11A6, 0xD43F,
0x5E52, 0x9BCB, 0x90F9, 0x5560, 0x869D, 0x4304, 0x4836, 0x8DAF,
0xAA55, 0x6FCC, 0x64FE, 0xA167, 0x729A, 0xB703, 0xBC31, 0x79A8,
0xA8EB, 0x6D72, 0x6640, 0xA3D9, 0x7024, 0xB5BD, 0xBE8F, 0x7B16,
0x5CEC, 0x9975, 0x9247, 0x57DE, 0x8423, 0x41BA, 0x4A88, 0x8F11,
0x057C, 0xC0E5, 0xCBD7, 0x0E4E, 0xDDB3, 0x182A, 0x1318, 0xD681,
0xF17B, 0x34E2, 0x3FD0, 0xFA49, 0x29B4, 0xEC2D, 0xE71F, 0x2286,
0xA213, 0x678A, 0x6CB8, 0xA921, 0x7ADC, 0xBF45, 0xB477, 0x71EE,
0x5614, 0x938D, 0x98BF, 0x5D26, 0x8EDB, 0x4B42, 0x4070, 0x85E9,
0x0F84, 0xCA1D, 0xC12F, 0x04B6, 0xD74B, 0x12D2, 0x19E0, 0xDC79,
0xFB83, 0x3E1A, 0x3528, 0xF0B1, 0x234C, 0xE6D5, 0xEDE7, 0x287E,
0xF93D, 0x3CA4, 0x3796, 0xF20F, 0x21F2, 0xE46B, 0xEF59, 0x2AC0,
0x0D3A, 0xC8A3, 0xC391, 0x0608, 0xD5F5, 0x106C, 0x1B5E, 0xDEC7,
0x54AA, 0x9133, 0x9A01, 0x5F98, 0x8C65, 0x49FC, 0x42CE, 0x8757,
0xA0AD, 0x6534, 0x6E06, 0xAB9F, 0x7862, 0xBDFB, 0xB6C9, 0x7350,
0x51D6, 0x944F, 0x9F7D, 0x5AE4, 0x8919, 0x4C80, 0x47B2, 0x822B,
0xA5D1, 0x6048, 0x6B7A, 0xAEE3, 0x7D1E, 0xB887, 0xB3B5, 0x762C,
0xFC41, 0x39D8, 0x32EA, 0xF773, 0x248E, 0xE117, 0xEA25, 0x2FBC,
0x0846, 0xCDDF, 0xC6ED, 0x0374, 0xD089, 0x1510, 0x1E22, 0xDBBB,
0x0AF8, 0xCF61, 0xC453, 0x01CA, 0xD237, 0x17AE, 0x1C9C, 0xD905,
0xFEFF, 0x3B66, 0x3054, 0xF5CD, 0x2630, 0xE3A9, 0xE89B, 0x2D02,
0xA76F, 0x62F6, 0x69C4, 0xAC5D, 0x7FA0, 0xBA39, 0xB10B, 0x7492,
0x5368, 0x96F1, 0x9DC3, 0x585A, 0x8BA7, 0x4E3E, 0x450C, 0x8095
};










void wakeup_idle(uint8_t total_ic)
{
  for (int i =0; i<total_ic; i++)
  {
    cs_low();
    HAL_Delay(1);
    cs_high();
  }
}

//Generic wakeup commannd to wake the LTC6813 from sleep
void wakeup_sleep(uint8_t total_ic)
{
  for (int i =0; i<total_ic; i++)
  {
    cs_low();
    HAL_Delay(1);
	cs_high();
  }
}

//Generic function to write 68xx commands. Function calculated PEC for tx_cmd data
void cmd_68(uint8_t tx_cmd[2])
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;

  cmd[0] = tx_cmd[0];
  cmd[1] =  tx_cmd[1];
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);
	cs_low();
  spi_write_array(4,cmd);
	cs_high();
}

//Generic function to write 68xx commands and write payload data. Function calculated PEC for tx_cmd data
void write_68(uint8_t total_ic , uint8_t tx_cmd[2], uint8_t data[])
{
  const uint8_t BYTES_IN_REG = 6;
  const uint8_t CMD_LEN = 4+(8*total_ic);
  uint8_t *cmd;
  uint16_t data_pec;
  uint16_t cmd_pec;
  uint8_t cmd_index;

  cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));
  cmd[0] = tx_cmd[0];
  cmd[1] = tx_cmd[1];
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);
  cmd_index = 4;
  for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--)       // executes for each LTC681x in daisy chain, this loops starts with
  {
    // the last IC on the stack. The first configuration written is
    // received by the last IC in the daisy chain

    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
      cmd[cmd_index] = data[((current_ic-1)*6)+current_byte];
      cmd_index = cmd_index + 1;
    }

    data_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &data[(current_ic-1)*6]);    // calculating the PEC for each Iss configuration register data
    cmd[cmd_index] = (uint8_t)(data_pec >> 8);
    cmd[cmd_index + 1] = (uint8_t)data_pec;
    cmd_index = cmd_index + 2;
  }


  cs_low();
  spi_write_array(CMD_LEN, cmd);
  cs_high();
  free(cmd);
}

//Generic function to write 68xx commands and read data. Function calculated PEC for tx_cmd data
int8_t read_68( uint8_t total_ic, uint8_t tx_cmd[2], uint8_t *rx_data)
{
  const uint8_t BYTES_IN_REG = 8;
  uint8_t cmd[4];
  uint8_t data[256];
  int8_t pec_error = 0;
  uint16_t cmd_pec;
  uint16_t data_pec;
  uint16_t received_pec;

  //data = (uint8_t *) malloc((8*total_ic)*sizeof(uint8_t)); // This is a problem because it can fail

  cmd[0] = tx_cmd[0];
  cmd[1] = tx_cmd[1
				  ];
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);


  cs_low();
  spi_write_read(cmd, 4, data, (BYTES_IN_REG*total_ic));         //Read the configuration data of all ICs on the daisy chain into
  cs_high();                          //rx_data[] array

  for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)       //executes for each LTC681x in the daisy chain and packs the data
  {
    //into the r_comm array as well as check the received Config data
    //for any bit errors
    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
      rx_data[(current_ic*8)+current_byte] = data[current_byte + (current_ic*BYTES_IN_REG)];
    }
    received_pec = (rx_data[(current_ic*8)+6]<<8) + rx_data[(current_ic*8)+7];
    data_pec = pec15_calc(6, &rx_data[current_ic*8]);
    if (received_pec != data_pec)
    {
      pec_error = -1;
    }
  }


  return(pec_error);
}


/*
  Calculates  and returns the CRC15
  */
uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
                    uint8_t *data //Array of data that will be used to calculate  a PEC
                   )
{
  uint16_t remainder,addr;

  remainder = 16;//initialize the PEC
  for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
  {
    addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address

    remainder = (remainder<<8)^crc15Table[addr];
  }
  return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

//Starts cell voltage conversion
void LTC681x_adcv(
  uint8_t MD, //ADC Mode
  uint8_t DCP, //Discharge Permit
  uint8_t CH //Cell Channels to be measured
)
{
  uint8_t cmd[4];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + 0x60 + (DCP<<4) + CH;
  cmd_68(cmd);
}


//Starts cell voltage and SOC conversion
void LTC681x_adcvsc(
  uint8_t MD, //ADC Mode
  uint8_t DCP //Discharge Permit
)
{
  uint8_t cmd[4];
  uint8_t md_bits;
  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits | 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits | 0x60 | (DCP<<4) | 0x07;
  cmd_68(cmd);

}

// Starts cell voltage  and GPIO 1&2 conversion
void LTC681x_adcvax(uint8_t MD, uint8_t DCP) {
    uint8_t cmd[4];
    uint8_t md_bits;
    md_bits = (MD & 0x02) >> 1;
    cmd[0] = md_bits | 0x04;
    md_bits = (MD & 0x01) << 7;
    cmd[1] = md_bits | (((DCP & 0x01) << 4) + 0x6F);
    cmd_68(cmd);
}
//Starts cell voltage overlap conversion
void LTC681x_adol(
  uint8_t MD, //ADC Mode
  uint8_t DCP //Discharge Permit
)
{
  uint8_t cmd[4];
  uint8_t md_bits;
  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + (DCP<<4) +0x01;
  cmd_68(cmd);
}

//Starts cell voltage self test conversion
void LTC681x_cvst(
  uint8_t MD, //ADC Mode
  uint8_t ST //Self Test
)
{
  uint8_t cmd[2];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + ((ST)<<5) +0x07;
  cmd_68(cmd);

}

//Start an Auxiliary Register Self Test Conversion
void LTC681x_axst(
  uint8_t MD, //ADC Mode
  uint8_t ST //Self Test
)
{
  uint8_t cmd[4];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + ((ST&0x03)<<5) +0x07;
  cmd_68(cmd);

}

//Start a Status Register Self Test Conversion
void LTC681x_statst(
  uint8_t MD, //ADC Mode
  uint8_t ST //Self Test
)
{
  uint8_t cmd[2];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + ((ST&0x03)<<5) +0x0F;
  cmd_68(cmd);

}

//Sends the poll adc command
uint8_t LTC681x_pladc()
{
  uint8_t cmd[4];
  uint8_t adc_state = 0xFF;
  uint16_t cmd_pec;

  cmd[0] = 0x07;
  cmd[1] = 0x14;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);


  cs_low();
  spi_write_array(4,cmd);
	//adc_state = spi_read_byte(0xFF);

  cs_high();
  return(adc_state);
}

//This function will block operation until the ADC has finished it's conversion
uint32_t LTC681x_pollAdc()
{
  uint32_t counter = 0;
  uint8_t finished = 0;
  uint8_t current_time = 0;
  uint8_t cmd[4];
  uint16_t cmd_pec;


  cmd[0] = 0x07;
  cmd[1] = 0x14;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  cs_low();
  spi_write_array(4,cmd);

  while ((counter<3600000)&&(finished == 0))
  {
    current_time = spi_read_byte(0xff);
    if (current_time>0)
    {
      finished = 1;
    }
    else
    {
      counter = counter + 10;
    }
  }

  cs_high();


  return(counter);
}

//Start a GPIO and Vref2 Conversion
void LTC681x_adax(
  uint8_t MD, //ADC Mode
  uint8_t CHG //GPIO Channels to be measured)
)
{
  uint8_t cmd[4];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] = md_bits + 0x60 + CHG ;
  cmd_68(cmd);

}

//Start an GPIO Redundancy test
void LTC681x_adaxd(
  uint8_t MD, //ADC Mode
  uint8_t CHG //GPIO Channels to be measured)
)
{
  uint8_t cmd[4];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] = md_bits + CHG ;
  cmd_68(cmd);
}

//Start a Status ADC Conversion
void LTC681x_adstat(
  uint8_t MD, //ADC Mode
  uint8_t CHST //GPIO Channels to be measured
)
{
  uint8_t cmd[4];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] = md_bits + 0x68 + CHST ;
  cmd_68(cmd);
}

// Start a Status register redundancy test Conversion
void LTC681x_adstatd(
  uint8_t MD, //ADC Mode
  uint8_t CHST //GPIO Channels to be measured
)
{
  uint8_t cmd[2];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] = md_bits + 0x08 + CHST ;
  cmd_68(cmd);

}


// Start an open wire Conversion
void LTC681x_adow(uint8_t MD, //ADC Mode
				  uint8_t PUP,//Pull up/Pull down current
				  uint8_t CH, //Channels
				  uint8_t DCP//Discharge Permit
				 )
{
	uint8_t cmd[2];
	uint8_t md_bits;

	md_bits = (MD & 0x02) >> 1;
	cmd[0] = md_bits + 0x02;
	md_bits = (MD & 0x01) << 7;
	cmd[1] =  md_bits + 0x28 + (PUP<<6) + CH+(DCP<<4);

	cmd_68(cmd);
}

// Reads the raw cell voltage register data
void LTC681x_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
                      uint8_t total_ic, //the number of ICs in the
                      uint8_t *data //An array of the unparsed cell codes
                     )
{
  const uint8_t REG_LEN = 8; //number of bytes in each ICs register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;

  if (reg == 1)     //1: RDCVA
  {
    cmd[1] = 0x04;
    cmd[0] = 0x00;
  }
  else if (reg == 2) //2: RDCVB
  {
    cmd[1] = 0x06;
    cmd[0] = 0x00;
  }
  else if (reg == 3) //3: RDCVC
  {
    cmd[1] = 0x08;
    cmd[0] = 0x00;
  }
  else if (reg == 4) //4: RDCVD
  {
    cmd[1] = 0x0A;
    cmd[0] = 0x00;
  }
  else if (reg == 5) //4: RDCVE
  {
    cmd[1] = 0x09;
    cmd[0] = 0x00;
  }
  else if (reg == 6) //4: RDCVF
  {
    cmd[1] = 0x0B;
    cmd[0] = 0x00;
  }


  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  cs_low();
  spi_write_read(cmd,4,data,(REG_LEN*total_ic));
  cs_high();

}

//helper function that parses voltage measurement registers
int8_t parse_cells(uint8_t current_ic, uint8_t cell_reg, uint8_t cell_data[], uint16_t *cell_codes, uint8_t *ic_pec)
{

  const uint8_t BYT_IN_REG = 6;
  const uint8_t CELL_IN_REG = 3;
  int8_t pec_error = 0;
  uint16_t parsed_cell;
  uint16_t received_pec;
  uint16_t data_pec;
  uint8_t data_counter = current_ic*NUM_RX_BYT; //data counter


  for (uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++)  // This loop parses the read back data into cell voltages, it
  {
    // loops once for each of the 3 cell voltage codes in the register

    parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);//Each cell code is received as two bytes and is combined to
    // create the parsed cell voltage code
    cell_codes[current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
    data_counter = data_counter + 2;                       //Because cell voltage codes are two bytes the data counter
    //must increment by two for each parsed cell code
  }

  received_pec = (cell_data[data_counter] << 8) | cell_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
  //after the 6 cell voltage data bytes
  data_pec = pec15_calc(BYT_IN_REG, &cell_data[(current_ic) * NUM_RX_BYT]);

  if (received_pec != data_pec)
  {
    pec_error = 1;                             //The pec_error variable is simply set negative if any PEC errors
    ic_pec[cell_reg-1]=1;
  }
  else
  {
    ic_pec[cell_reg-1]=0;
  }
  data_counter=data_counter+2;
  return(pec_error);
}

/*
The function reads a single GPIO voltage register and stores thre read data
in the *data point as a byte array. This function is rarely used outside of
the LTC6811_rdaux() command.
*/
void LTC681x_rdaux_reg(uint8_t reg, //Determines which GPIO voltage register is read back
                       uint8_t total_ic, //The number of ICs in the system
                       uint8_t *data //Array of the unparsed auxiliary codes
                      )
{
  const uint8_t REG_LEN = 8; // number of bytes in the register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;


  if (reg == 1)     //Read back auxiliary group A
  {
    cmd[1] = 0x0C;
    cmd[0] = 0x00;
  }
  else if (reg == 2)  //Read back auxiliary group B
  {
    cmd[1] = 0x0e;
    cmd[0] = 0x00;
  }
  else if (reg == 3)  //Read back auxiliary group C
  {
    cmd[1] = 0x0D;
    cmd[0] = 0x00;
  }
  else if (reg == 4)  //Read back auxiliary group D
  {
    cmd[1] = 0x0F;
    cmd[0] = 0x00;
  }
  else          //Read back auxiliary group A
  {
    cmd[1] = 0x0C;
    cmd[0] = 0x00;
  }

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  cs_low();
  spi_write_read(cmd,4,data,(REG_LEN*total_ic));
  cs_high();

}

/*
The function reads a single stat  register and stores the read data
in the *data point as a byte array. This function is rarely used outside of
the LTC6811_rdstat() command.
*/
void LTC681x_rdstat_reg(uint8_t reg, //Determines which stat register is read back
                        uint8_t total_ic, //The number of ICs in the system
                        uint8_t *data //Array of the unparsed stat codes
                       )
{
  const uint8_t REG_LEN = 8; // number of bytes in the register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;


  if (reg == 1)     //Read back statiliary group A
  {
    cmd[1] = 0x10;
    cmd[0] = 0x00;
  }
  else if (reg == 2)  //Read back statiliary group B
  {
    cmd[1] = 0x12;
    cmd[0] = 0x00;
  }

  else          //Read back statiliary group A
  {
    cmd[1] = 0x10;
    cmd[0] = 0x00;
  }

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  cs_low();
  spi_write_read(cmd,4,data,(REG_LEN*total_ic));
  cs_high();

}

/*
The command clears the cell voltage registers and intiallizes
all values to 1. The register will read back hexadecimal 0xFF
after the command is sent.
*/
void LTC681x_clrcell()
{
  uint8_t cmd[2]= {0x07 , 0x11};
  cmd_68(cmd);
}


/*
The command clears the Auxiliary registers and initializes
all values to 1. The register will read back hexadecimal 0xFF
after the command is sent.
*/
void LTC681x_clraux()
{
  uint8_t cmd[2]= {0x07 , 0x12};
  cmd_68(cmd);
}


/*
The command clears the Stat registers and intiallizes
all values to 1. The register will read back hexadecimal 0xFF
after the command is sent.

*/
void LTC681x_clrstat()
{
  uint8_t cmd[2]= {0x07 , 0x13};
  cmd_68(cmd);
}
/*
The command clears the Sctrl registers and initializes
all values to 0. The register will read back hexadecimal 0x00
after the command is sent.
*/
void LTC681x_clrsctrl()
{
  uint8_t cmd[2]= {0x00 , 0x18};
  cmd_68(cmd);
}
//Starts the Mux Decoder diagnostic self test
void LTC681x_diagn()
{
  uint8_t cmd[2] = {0x07 , 0x15};
  cmd_68(cmd);
}

//Reads and parses the LTC681x cell voltage registers.
uint8_t LTC681x_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
                     uint8_t total_ic, // the number of ICs in the system
                     cell_asic ic[] // Array of the parsed cell codes
                    )
{
  int8_t pec_error = 0;
  uint8_t *cell_data;
  uint8_t c_ic = 0;
  cell_data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));
  //uint8_t cell_data[NUM_RX_BYT * total_ic];
  if (reg == 0)
  {
    for (uint8_t cell_reg = 1; cell_reg<ic[0].ic_reg.num_cv_reg+1; cell_reg++)                   //executes once for each of the LTC6811 cell voltage registers
    {
      LTC681x_rdcv_reg(cell_reg, total_ic,cell_data );
      for (int current_ic = 0; current_ic<total_ic; current_ic++)
      {
        if (ic->isospi_reverse == false)
        {
          c_ic = current_ic;
        }
        else
        {
          c_ic = total_ic - current_ic - 1;
        }
        pec_error = pec_error + parse_cells(current_ic,cell_reg, cell_data,
                                            &ic[c_ic].cells.c_codes[0],
                                            &ic[c_ic].cells.pec_match[0]);
      }
    }
  }

  else
  {
    LTC681x_rdcv_reg(reg, total_ic,cell_data);

    for (int current_ic = 0; current_ic<total_ic; current_ic++)
    {
      if (ic->isospi_reverse == false)
      {
        c_ic = current_ic;
      }
      else
      {
        c_ic = total_ic - current_ic - 1;
      }
      pec_error = pec_error + parse_cells(current_ic,reg, &cell_data[8*c_ic],
                                          &ic[c_ic].cells.c_codes[0],
                                          &ic[c_ic].cells.pec_match[0]);
    }
  }
  LTC681x_check_pec(total_ic,CELL,ic);
  free(cell_data);
  return(pec_error);
}



/*
The function is used
to read the  parsed GPIO codes of the LTC6811. This function will send the requested
read commands parse the data and store the gpio voltages in aux_codes variable
*/
int8_t LTC681x_rdaux(uint8_t reg, //Determines which GPIO voltage register is read back.
                     uint8_t total_ic,//the number of ICs in the system
                     cell_asic ic[]//A two dimensional array of the gpio voltage codes.
                    )
{
  //uint8_t *data;
  int8_t pec_error = 0;
  uint8_t c_ic =0;
  //data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));
  uint8_t data[NUM_RX_BYT * total_ic];

  if (reg == 0)
  {
    for (uint8_t gpio_reg = 1; gpio_reg<ic[0].ic_reg.num_gpio_reg+1; gpio_reg++)                 //executes once for each of the LTC6811 aux voltage registers
    {
      LTC681x_rdaux_reg(gpio_reg, total_ic,data);                 //Reads the raw auxiliary register data into the data[] array
      for (int current_ic = 0; current_ic<total_ic; current_ic++)
      {
        if (ic->isospi_reverse == false)
        {
          c_ic = current_ic;
        }
        else
        {
          c_ic = total_ic - current_ic - 1;
        }
        pec_error = parse_cells(current_ic,gpio_reg, data,
                                &ic[c_ic].aux.a_codes[0],
                                &ic[c_ic].aux.pec_match[0]);

      }
    }
  }
  else
  {
    LTC681x_rdaux_reg(reg, total_ic, data);

    for (int current_ic = 0; current_ic<total_ic; current_ic++)
    {
      if (ic->isospi_reverse == false)
      {
        c_ic = current_ic;
      }
      else
      {
        c_ic = total_ic - current_ic - 1;
      }
      pec_error = parse_cells(current_ic,reg, data,
                              &ic[c_ic].aux.a_codes[0],
                              &ic[c_ic].aux.pec_match[0]);
    }

  }
  LTC681x_check_pec(total_ic,AUX,ic);
  //free(data);
  return (pec_error);
}

// Reads and parses the LTC681x stat registers.
int8_t LTC681x_rdstat(uint8_t reg, //Determines which Stat  register is read back.
                      uint8_t total_ic,//the number of ICs in the system
                      cell_asic ic[]
                     )

{

  const uint8_t BYT_IN_REG = 6;
  const uint8_t GPIO_IN_REG = 3;

  uint8_t *data;
  uint8_t data_counter = 0;
  int8_t pec_error = 0;
  uint16_t parsed_stat;
  uint16_t received_pec;
  uint16_t data_pec;
  uint8_t c_ic = 0;
  data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));

  if (reg == 0)
  {

    for (uint8_t stat_reg = 1; stat_reg< 3; stat_reg++)                      //executes once for each of the LTC6811 stat voltage registers
    {
      data_counter = 0;
      LTC681x_rdstat_reg(stat_reg, total_ic,data);                            //Reads the raw statiliary register data into the data[] array

      for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)      // executes for every LTC6811 in the daisy chain
      {
        if (ic->isospi_reverse == false)
        {
          c_ic = current_ic;
        }
        else
        {
          c_ic = total_ic - current_ic - 1;
        }
        // current_ic is used as the IC counter
        if (stat_reg ==1)
        {
          for (uint8_t current_gpio = 0; current_gpio< GPIO_IN_REG; current_gpio++) // This loop parses the read back data into GPIO voltages, it
          {
            // loops once for each of the 3 gpio voltage codes in the register

            parsed_stat = data[data_counter] + (data[data_counter+1]<<8);              //Each gpio codes is received as two bytes and is combined to
            ic[c_ic].stat.stat_codes[current_gpio] = parsed_stat;
            data_counter=data_counter+2;                                               //Because gpio voltage codes are two bytes the data counter

          }
        }
        else if (stat_reg == 2)
        {
          parsed_stat = data[data_counter] + (data[data_counter+1]<<8);              //Each gpio codes is received as two bytes and is combined to
          data_counter = data_counter +2;
          ic[c_ic].stat.stat_codes[3] = parsed_stat;
          ic[c_ic].stat.flags[0] = data[data_counter++];
          ic[c_ic].stat.flags[1] = data[data_counter++];
          ic[c_ic].stat.flags[2] = data[data_counter++];
          ic[c_ic].stat.mux_fail[0] = (data[data_counter] & 0x02)>>1;
          ic[c_ic].stat.thsd[0] = data[data_counter++] & 0x01;
        }

        received_pec = (data[data_counter]<<8)+ data[data_counter+1];          //The received PEC for the current_ic is transmitted as the 7th and 8th
        //after the 6 gpio voltage data bytes
        data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);

        if (received_pec != data_pec)
        {
          pec_error = -1; //The pec_error variable is simply set negative if any PEC errors
          ic[c_ic].stat.pec_match[stat_reg-1]=1;
          //are detected in the received serial data
        }
        else
        {
          ic[c_ic].stat.pec_match[stat_reg-1]=0;
        }

        data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
        //must be incremented by 2 bytes to point to the next ICs gpio voltage data
      }


    }

  }
  else
  {

    LTC681x_rdstat_reg(reg, total_ic, data);
    for (int current_ic = 0 ; current_ic < total_ic; current_ic++)            // executes for every LTC6811 in the daisy chain
    {
      // current_ic is used as an IC counter
      if (ic->isospi_reverse == false)
      {
        c_ic = current_ic;
      }
      else
      {
        c_ic = total_ic - current_ic - 1;
      }
      if (reg ==1)
      {
        for (uint8_t current_gpio = 0; current_gpio< GPIO_IN_REG; current_gpio++) // This loop parses the read back data into GPIO voltages, it
        {
          // loops once for each of the 3 gpio voltage codes in the register
          parsed_stat = data[data_counter] + (data[data_counter+1]<<8);              //Each gpio codes is received as two bytes and is combined to
          // create the parsed gpio voltage code

          ic[c_ic].stat.stat_codes[current_gpio] = parsed_stat;
          data_counter=data_counter+2;                        //Because gpio voltage codes are two bytes the data counter
          //must increment by two for each parsed gpio voltage code

        }
      }
      else if (reg == 2)
      {
        parsed_stat = data[data_counter] + (data[data_counter + 1] << 8);
        data_counter += 2;   //Each gpio codes is received as two bytes and is combined to
        ic[c_ic].stat.stat_codes[3] = parsed_stat;
        ic[c_ic].stat.flags[0] = data[data_counter++];
        ic[c_ic].stat.flags[1] = data[data_counter++];
        ic[c_ic].stat.flags[2] = data[data_counter++];
        ic[c_ic].stat.mux_fail[0] = (data[data_counter] & 0x02)>>1;
        ic[c_ic].stat.thsd[0] = data[data_counter++] & 0x01;
      }


      received_pec = (data[data_counter]<<8)+ data[data_counter+1];          //The received PEC for the current_ic is transmitted as the 7th and 8th
      //after the 6 gpio voltage data bytes
      data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);
      if (received_pec != data_pec)
      {
        pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
        ic[c_ic].stat.pec_match[reg-1]=1;

      }

      data_counter=data_counter+2;
    }
  }
  LTC681x_check_pec(total_ic,STAT,ic);
  free(data);
  return (pec_error);
}

//Write the LTC681x CFGRA
void LTC681x_wrcfg(uint8_t total_ic, //The number of ICs being written to
                   cell_asic ic[]
                  )
{
  uint8_t cmd[2] = {0x00 , 0x01} ;
  uint8_t write_buffer[256];
  uint8_t write_count = 0;
  uint8_t c_ic = 0;
  for (uint8_t current_ic = 0; current_ic<total_ic; current_ic++)
  {
    if (ic->isospi_reverse == true)
    {
      c_ic = current_ic;
    }
    else
    {
      c_ic = total_ic - current_ic - 1;
    }

    for (uint8_t data = 0; data<6; data++)
    {
      write_buffer[write_count] = ic[c_ic].config.tx_data[data];
      write_count++;
    }
  }
  write_68(total_ic, cmd, write_buffer);
}

//Write the LTC681x CFGRB
void LTC681x_wrcfgb(uint8_t total_ic, //The number of ICs being written to
                    cell_asic ic[]
                   )
{
  uint8_t cmd[2] = {0x00 , 0x24} ;
  uint8_t write_buffer[256];
  uint8_t write_count = 0;
  uint8_t c_ic = 0;
  for (uint8_t current_ic = 0; current_ic<total_ic; current_ic++)
  {
    if (ic->isospi_reverse == true)
    {
      c_ic = current_ic;
    }
    else
    {
      c_ic = total_ic - current_ic - 1;
    }

    for (uint8_t data = 0; data<6; data++)
    {
      write_buffer[write_count] = ic[c_ic].configb.tx_data[data];
      write_count++;
    }
  }
  write_68(total_ic, cmd, write_buffer);
}

//Read CFGA
int8_t LTC681x_rdcfg(uint8_t total_ic, //Number of ICs in the system
                     cell_asic ic[]
                    )
{
  uint8_t cmd[2]= {0x00 , 0x02};
  uint8_t read_buffer[256];
  int8_t pec_error = 0;
  uint16_t data_pec;
  uint16_t calc_pec;
  uint8_t c_ic = 0;
  pec_error = read_68(total_ic, cmd, read_buffer);
  for (uint8_t current_ic = 0; current_ic<total_ic; current_ic++)
  {
    if (ic->isospi_reverse == false)
    {
      c_ic = current_ic;
    }
    else
    {
      c_ic = total_ic - current_ic - 1;
    }

    for (int byte=0; byte<8; byte++)
    {
      ic[c_ic].config.rx_data[byte] = read_buffer[byte+(8*current_ic)];
    }
    calc_pec = pec15_calc(6,&read_buffer[8*current_ic]);
    data_pec = read_buffer[7+(8*current_ic)] | (read_buffer[6+(8*current_ic)]<<8);
    if (calc_pec != data_pec )
    {
      ic[c_ic].config.rx_pec_match = 1;
    }
    else ic[c_ic].config.rx_pec_match = 0;
  }
  LTC681x_check_pec(total_ic,CFGRA,ic);
  return(pec_error);
}

//Reads CFGB
int8_t LTC681x_rdcfgb(uint8_t total_ic, //Number of ICs in the system
                      cell_asic ic[]
                     )
{
  uint8_t cmd[2]= {0x00 , 0x26};
  uint8_t read_buffer[256];
  int8_t pec_error = 0;
  uint16_t data_pec;
  uint16_t calc_pec;
  uint8_t c_ic = 0;
  pec_error = read_68(total_ic, cmd, read_buffer);
  for (uint8_t current_ic = 0; current_ic<total_ic; current_ic++)
  {
    if (ic->isospi_reverse == false)
    {
      c_ic = current_ic;
    }
    else
    {
      c_ic = total_ic - current_ic - 1;
    }

    for (int byte=0; byte<8; byte++)
    {
      ic[c_ic].configb.rx_data[byte] = read_buffer[byte+(8*current_ic)];
    }
    calc_pec = pec15_calc(6,&read_buffer[8*current_ic]);
    data_pec = read_buffer[7+(8*current_ic)] | (read_buffer[6+(8*current_ic)]<<8);
    if (calc_pec != data_pec )
    {
      ic[c_ic].configb.rx_pec_match = 1;
    }
    else ic[c_ic].configb.rx_pec_match = 0;
  }
  LTC681x_check_pec(total_ic,CFGRB,ic);
  return(pec_error);
}

//Looks up the result pattern for digital filter self test
uint16_t LTC681x_st_lookup(uint8_t MD, //ADC Mode
						   uint8_t ST, //Self Test
						   bool adcopt // ADCOPT bit in the configuration register
						  )
{
	uint16_t test_pattern = 0;

    if (MD == 1)
    {
		if ( adcopt == false)
		{
			if (ST == 1)
			{
				test_pattern = 0x9565;
			}
			else
			{
				test_pattern = 0x6A9A;
			}
		}
		else
		{
			if (ST == 1)
			{
				test_pattern = 0x9553;
			}
			else
			{
				test_pattern = 0x6AAC;
			}
		}
    }
    else
    {
		if (ST == 1)
		{
		   test_pattern = 0x9555;
		}
		else
		{
		   test_pattern = 0x6AAA;
		}
    }
    return(test_pattern);
}
//Clears all of the DCC bits in the configuration registers
void clear_discharge(uint8_t total_ic, cell_asic ic[])
{
  for (int i=0; i<total_ic; i++)
  {
    ic[i].config.tx_data[4] = 0;
    ic[i].config.tx_data[5] = 0;
  }
}

// Runs the Digital Filter Self Test
int16_t LTC681x_run_cell_adc_st(uint8_t adc_reg, // Type of register
								uint8_t total_ic, // Number of ICs in the daisy chain
								cell_asic *ic, // A two dimensional array that will store the data
								uint8_t md, // ADC Mode
								bool adcopt // ADCOPT bit in the configuration register
								)
{
	int16_t error = 0;
	uint16_t expected_result = 0;

	for (int self_test = 1; self_test<3; self_test++)
	{
		expected_result = LTC681x_st_lookup(md,self_test,adcopt);
		wakeup_idle(total_ic);

		switch (adc_reg)
		{
		  case CELL:
			  wakeup_idle(total_ic);
			  LTC681x_clrcell();
			  LTC681x_cvst(md,self_test);
			  LTC681x_pollAdc();

			  wakeup_idle(total_ic);
			  error = LTC681x_rdcv(0, total_ic,ic);
			  for (int cic = 0; cic < total_ic; cic++)
				{
				  for (int channel=0; channel< ic[cic].ic_reg.cell_channels; channel++)
				  {

					if (ic[cic].cells.c_codes[channel] != expected_result)
					{
					  error = error+1;
					}
				  }
				}
			break;
		  case AUX:
			  error = 0;
			  wakeup_idle(total_ic);
			  LTC681x_clraux();
			  LTC681x_axst(md,self_test);
			  LTC681x_pollAdc();

			  wakeup_idle(total_ic);
			  LTC681x_rdaux(0, total_ic,ic);
			  for (int cic = 0; cic < total_ic; cic++)
				{
				  for (int channel=0; channel< ic[cic].ic_reg.aux_channels; channel++)
				  {

					if (ic[cic].aux.a_codes[channel] != expected_result)
					{
					  error = error+1;
					}
				  }
				}
			break;
		  case STAT:
			  wakeup_idle(total_ic);
			  LTC681x_clrstat();
			  LTC681x_statst(md,self_test);
			  LTC681x_pollAdc();

			  wakeup_idle(total_ic);
			  error = LTC681x_rdstat(0,total_ic,ic);
			  for (int cic = 0; cic < total_ic; cic++)
				{
				  for (int channel=0; channel< ic[cic].ic_reg.stat_channels; channel++)
				  {
					if (ic[cic].stat.stat_codes[channel] != expected_result)
					{
					  error = error+1;
					}
				  }
				}
			break;

		  default:
			error = -1;
			break;
		}
	}

	return(error);
}
//runs the redundancy self test
int16_t LTC681x_run_adc_redundancy_st(uint8_t adc_mode, uint8_t adc_reg, uint8_t total_ic, cell_asic ic[])
{
  int16_t error = 0;
  for (int self_test = 1; self_test<3; self_test++)
  {
    wakeup_idle(total_ic);
    switch (adc_reg)
    {
      case AUX:
        LTC681x_clraux();
        LTC681x_adaxd(adc_mode,AUX_CH_ALL);
        LTC681x_pollAdc();
        wakeup_idle(total_ic);
        error = LTC681x_rdaux(0, total_ic,ic);
        for (int cic = 0; cic < total_ic; cic++)
        {
          for (int channel=0; channel< ic[cic].ic_reg.aux_channels; channel++)
          {
            if (ic[cic].aux.a_codes[channel] >= 65280)
            {
              error = error+1;
            }
          }
        }
        break;
      case STAT:
        LTC681x_clrstat();
        LTC681x_adstatd(adc_mode,STAT_CH_ALL);
        LTC681x_pollAdc();
        wakeup_idle(total_ic);
        error = LTC681x_rdstat(0,total_ic,ic);
        for (int cic = 0; cic < total_ic; cic++)
        {
          for (int channel=0; channel< ic[cic].ic_reg.stat_channels; channel++)
          {
            if (ic[cic].stat.stat_codes[channel] >= 65280)
            {
              error = error+1;
            }
          }
        }
        break;

      default:
        error = -1;
        break;
    }
  }
  return(error);
}

//Runs the datasheet algorithm for open wire

// Runs the ADC overlap test for the IC
uint16_t LTC681x_run_adc_overlap(uint8_t total_ic, cell_asic ic[])
{
  uint16_t error = 0;
  int32_t measure_delta =0;
  int16_t failure_pos_limit = 20;
  int16_t failure_neg_limit = -20;
  wakeup_idle(total_ic);
  LTC681x_adol(MD_7KHZ_3KHZ,DCP_DISABLED);
  LTC681x_pollAdc();
  wakeup_idle(total_ic);
  error = LTC681x_rdcv(0, total_ic,ic);
  for (int cic = 0; cic<total_ic; cic++)
  {
    measure_delta = (int32_t)ic[cic].cells.c_codes[6]-(int32_t)ic[cic].cells.c_codes[7];
    if ((measure_delta>failure_pos_limit) || (measure_delta<failure_neg_limit))
    {
      error = error | (1<<(cic-1));
    }
  }
  return(error);
}

//Helper function that increments PEC counters
void LTC681x_check_pec(uint8_t total_ic,uint8_t reg, cell_asic ic[])
{
  switch (reg)
  {
    case CFGRA:
      for (int current_ic = 0 ; current_ic < total_ic; current_ic++)
      {
        ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + ic[current_ic].config.rx_pec_match;
        ic[current_ic].crc_count.cfgr_pec = ic[current_ic].crc_count.cfgr_pec + ic[current_ic].config.rx_pec_match;
      }
      break;

    case CFGRB:
      for (int current_ic = 0 ; current_ic < total_ic; current_ic++)
      {
        ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + ic[current_ic].configb.rx_pec_match;
        ic[current_ic].crc_count.cfgr_pec = ic[current_ic].crc_count.cfgr_pec + ic[current_ic].configb.rx_pec_match;
      }
      break;
    case CELL:
      for (int current_ic = 0 ; current_ic < total_ic; current_ic++)
      {
        for (int i=0; i<ic[0].ic_reg.num_cv_reg; i++)
        {
          ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + ic[current_ic].cells.pec_match[i];
          ic[current_ic].crc_count.cell_pec[i] = ic[current_ic].crc_count.cell_pec[i] + ic[current_ic].cells.pec_match[i];
        }
      }
      break;
    case AUX:
      for (int current_ic = 0 ; current_ic < total_ic; current_ic++)
      {
        for (int i=0; i<ic[0].ic_reg.num_gpio_reg; i++)
        {
          ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + (ic[current_ic].aux.pec_match[i]);
          ic[current_ic].crc_count.aux_pec[i] = ic[current_ic].crc_count.aux_pec[i] + (ic[current_ic].aux.pec_match[i]);
        }
      }

      break;
    case STAT:
      for (int current_ic = 0 ; current_ic < total_ic; current_ic++)
      {

        for (int i=0; i<ic[0].ic_reg.num_stat_reg-1; i++)
        {
          ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + ic[current_ic].stat.pec_match[i];
          ic[current_ic].crc_count.stat_pec[i] = ic[current_ic].crc_count.stat_pec[i] + ic[current_ic].stat.pec_match[i];
        }
      }
      break;
    default:
      break;
  }
}

//Helper Function to reset PEC counters
void LTC681x_reset_crc_count(uint8_t total_ic, cell_asic ic[])
{
  for (int current_ic = 0 ; current_ic < total_ic; current_ic++)
  {
    ic[current_ic].crc_count.pec_count = 0;
    ic[current_ic].crc_count.cfgr_pec = 0;
    for (int i=0; i<6; i++)
    {
      ic[current_ic].crc_count.cell_pec[i]=0;

    }
    for (int i=0; i<4; i++)
    {
      ic[current_ic].crc_count.aux_pec[i]=0;
    }
    for (int i=0; i<2; i++)
    {
      ic[current_ic].crc_count.stat_pec[i]=0;
    }
  }
}

//Helper function to intialize CFG variables.
void LTC681x_init_cfg(uint8_t total_ic, cell_asic ic[])
{
  bool REFON = true;
  bool ADCOPT = false;
  bool gpioBits[5] = {true,true,true,true,true};
  bool dccBits[12] = {false,false,false,false,false,false,false,false,false,false,false,false};
  for (uint8_t current_ic = 0; current_ic<total_ic; current_ic++)
  {
    for (int j =0; j<6; j++)
    {
      ic[current_ic].config.tx_data[j] = 0;
      ic[current_ic].configb.tx_data[j] = 0;
    }
    LTC681x_set_cfgr(current_ic ,ic,REFON,ADCOPT,gpioBits,dccBits);

  }
}

//Helper function to set CFGR variable
void LTC681x_set_cfgr(uint8_t nIC, cell_asic ic[], bool refon, bool adcopt, bool gpio[5],bool dcc[12])
{
  LTC681x_set_cfgr_refon(nIC,ic,refon);
  LTC681x_set_cfgr_adcopt(nIC,ic,adcopt);
  LTC681x_set_cfgr_gpio(nIC,ic,gpio);
  LTC681x_set_cfgr_dis(nIC,ic,dcc);
}

//Helper function to set the REFON bit
void LTC681x_set_cfgr_refon(uint8_t nIC, cell_asic ic[], bool refon)
{
  if (refon) ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]|0x04;
  else ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]&0xFB;
}

//Helper function to set the adcopt bit
void LTC681x_set_cfgr_adcopt(uint8_t nIC, cell_asic ic[], bool adcopt)
{
  if (adcopt) ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]|0x01;
  else ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]&0xFE;
}

//Helper function to set GPIO bits
void LTC681x_set_cfgr_gpio(uint8_t nIC, cell_asic ic[],bool gpio[])
{
  for (int i =0; i<5; i++)
  {
    if (gpio[i])ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]|(0x01<<(i+3));
    else ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]&(~(0x01<<(i+3)));
  }
}

//Helper function to control discharge
void LTC681x_set_cfgr_dis(uint8_t nIC, cell_asic ic[],bool dcc[])
{
  for (int i =0; i<8; i++)
  {
    if (dcc[i])ic[nIC].config.tx_data[4] = ic[nIC].config.tx_data[4]|(0x01<<i);
    else ic[nIC].config.tx_data[4] = ic[nIC].config.tx_data[4]& (~(0x01<<i));
  }
  for (int i =0; i<4; i++)
  {
    if (dcc[i+8])ic[nIC].config.tx_data[5] = ic[nIC].config.tx_data[5]|(0x01<<i);
    else ic[nIC].config.tx_data[5] = ic[nIC].config.tx_data[5]&(~(0x01<<i));
  }
}

//Helper Function to set uv value in CFG register
void LTC681x_set_cfgr_uv(uint8_t nIC, cell_asic ic[],uint16_t uv)
{
  uint16_t tmp = (uv/16)-1;
  ic[nIC].config.tx_data[1] = 0x00FF & tmp;
  ic[nIC].config.tx_data[2] = ic[nIC].config.tx_data[2]&0xF0;
  ic[nIC].config.tx_data[2] = ic[nIC].config.tx_data[2]|((0x0F00 & tmp)>>8);
}

//helper function to set OV value in CFG register
void LTC681x_set_cfgr_ov(uint8_t nIC, cell_asic ic[],uint16_t ov)
{
  uint16_t tmp = (ov/16);
  ic[nIC].config.tx_data[3] = 0x00FF & (tmp>>4);
  ic[nIC].config.tx_data[2] = ic[nIC].config.tx_data[2]&0x0F;
  ic[nIC].config.tx_data[2] = ic[nIC].config.tx_data[2]|((0x000F & tmp)<<4);
}

//Writes the comm register
void LTC681x_wrcomm(uint8_t total_ic, //The number of ICs being written to
                    cell_asic ic[]
                   )
{
  uint8_t cmd[2]= {0x07 , 0x21};
  uint8_t write_buffer[256];
  uint8_t write_count = 0;
  uint8_t c_ic = 0;
  for (uint8_t current_ic = 0; current_ic<total_ic; current_ic++)
  {
    if (ic->isospi_reverse == true)
    {
      c_ic = current_ic;
    }
    else
    {
      c_ic = total_ic - current_ic - 1;
    }

    for (uint8_t data = 0; data<6; data++)
    {
      write_buffer[write_count] = ic[c_ic].com.tx_data[data];
      write_count++;
    }
  }
  write_68(total_ic, cmd, write_buffer);
}

/*
Reads COMM registers of a LTC6811 daisy chain
*/
int8_t LTC681x_rdcomm(uint8_t total_ic, //Number of ICs in the system
                      cell_asic ic[]
                     )
{
  uint8_t cmd[2]= {0x07 , 0x22};
  uint8_t read_buffer[256];
  int8_t pec_error = 0;
  uint16_t data_pec;
  uint16_t calc_pec;
  uint8_t c_ic=0;
  pec_error = read_68(total_ic, cmd, read_buffer);
  for (uint8_t current_ic = 0; current_ic<total_ic; current_ic++)
  {
    if (ic->isospi_reverse == false)
    {
      c_ic = current_ic;
    }
    else
    {
      c_ic = total_ic - current_ic - 1;
    }

    for (int byte=0; byte<8; byte++)
    {
      ic[c_ic].com.rx_data[byte] = read_buffer[byte+(8*current_ic)];
    }
    calc_pec = pec15_calc(6,&read_buffer[8*current_ic]);
    data_pec = read_buffer[7+(8*current_ic)] | (read_buffer[6+(8*current_ic)]<<8);
    if (calc_pec != data_pec )
    {
      ic[c_ic].com.rx_pec_match = 1;
    }
    else ic[c_ic].com.rx_pec_match = 0;
  }
  return(pec_error);
}

/*
Shifts data in COMM register out over LTC6811 SPI/I2C port
*/
void LTC681x_stcomm()
{

  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = 0x07;
  cmd[1] = 0x23;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  cs_low();
  spi_write_array(4,cmd);
  for (int i = 0; i<9; i++)
  {
    spi_read_byte(0xFF);
  }
  cs_high();

}

// Writes the pwm register
void LTC681x_wrpwm(uint8_t total_ic,
                   uint8_t pwmReg,
                   cell_asic ic[]
                  )
{
  uint8_t cmd[2];
  uint8_t write_buffer[256];
  uint8_t write_count = 0;
  uint8_t c_ic = 0;
  if (pwmReg == 0)
  {
    cmd[0] = 0x00;
    cmd[1] = 0x20;
  }
  else
  {
    cmd[0] = 0x00;
    cmd[1] = 0x1C;
  }

  for (uint8_t current_ic = 0; current_ic<total_ic; current_ic++)
  {
    if (ic->isospi_reverse == true)
    {
      c_ic = current_ic;
    }
    else
    {
      c_ic = total_ic - current_ic - 1;
    }
    for (uint8_t data = 0; data<6; data++)
    {
      write_buffer[write_count] = ic[c_ic].pwm.tx_data[data];
      write_count++;
    }
  }
  write_68(total_ic, cmd, write_buffer);
}


/*
Reads pwm registers of a LTC6811 daisy chain
*/
int8_t LTC681x_rdpwm(uint8_t total_ic, //Number of ICs in the system
                     uint8_t pwmReg,
                     cell_asic ic[]
                    )
{
  const uint8_t BYTES_IN_REG = 8;

  uint8_t cmd[4];
  uint8_t read_buffer[256];
  int8_t pec_error = 0;
  uint16_t data_pec;
  uint16_t calc_pec;
  uint8_t c_ic = 0;

  if (pwmReg == 0)
  {
    cmd[0] = 0x00;
    cmd[1] = 0x22;
  }
  else
  {
    cmd[0] = 0x00;
    cmd[1] = 0x1E;
  }


  pec_error = read_68(total_ic, cmd, read_buffer);
  for (uint8_t current_ic =0; current_ic<total_ic; current_ic++)
  {
    if (ic->isospi_reverse == false)
    {
      c_ic = current_ic;
    }
    else
    {
      c_ic = total_ic - current_ic - 1;
    }
    for (int byte=0; byte<8; byte++)
    {
      ic[c_ic].pwm.rx_data[byte] = read_buffer[byte+(8*current_ic)];
    }
    calc_pec = pec15_calc(6,&read_buffer[8*current_ic]);
    data_pec = read_buffer[7+(8*current_ic)] | (read_buffer[6+(8*current_ic)]<<8);
    if (calc_pec != data_pec )
    {
      ic[c_ic].pwm.rx_pec_match = 1;
    }
    else ic[c_ic].pwm.rx_pec_match = 0;
  }
  return(pec_error);
}




void LTC681x_axow(uint8_t MD, //ADC Mode
				  uint8_t PUP //Pull up/Pull down current
				 )
{
	uint8_t cmd[2];
	uint8_t md_bits;

	md_bits = (MD & 0x02) >> 1;
	cmd[0] = md_bits + 0x04;
	md_bits = (MD & 0x01) << 7;
	cmd[1] =  md_bits + 0x10+ (PUP<<6) ;//+ CH;

	cmd_68(cmd);
}



void LTC681x_clear_discharge(uint8_t total_ic, // Number of ICs in the daisy chain
							 cell_asic *ic // A two dimensional array that will store the data
							 )
{
	for (int i=0; i<total_ic; i++)
	{
	   ic[i].config.tx_data[4] = 0;
	   ic[i].config.tx_data[5] =ic[i].config.tx_data[5]&(0xF0);
	   ic[i].configb.tx_data[0]=ic[i].configb.tx_data[0]&(0x0F);
	   ic[i].configb.tx_data[1]=ic[i].configb.tx_data[1]&(0xF0);
	}
}




int8_t LTC681x_rdsctrl(uint8_t total_ic, // Number of ICs in the daisy chain
                       uint8_t sctrl_reg, // The Sctrl Register to be written A or B
                       cell_asic *ic // A two dimensional array that the function stores the read data
                      )
{
    uint8_t cmd[4];
    uint8_t read_buffer[256];
    int8_t pec_error = 0;
    uint16_t data_pec;
    uint16_t calc_pec;
    uint8_t c_ic = 0;

    if (sctrl_reg == 0)
    {
      cmd[0] = 0x00;
      cmd[1] = 0x16;
	  }
    else
    {
      cmd[0] = 0x00;
      cmd[1] = 0x1E;
	}

    pec_error = read_68(total_ic, cmd, read_buffer);

    for(uint8_t current_ic =0; current_ic<total_ic; current_ic++)
    {
        if(ic->isospi_reverse == false){c_ic = current_ic;}
        else{c_ic = total_ic - current_ic - 1;}


        for(int byte=0; byte<8;byte++)
        {
            ic[c_ic].sctrl.rx_data[byte] = read_buffer[byte+(8*current_ic)];
        }

        calc_pec = pec15_calc(6,&read_buffer[8*current_ic]);
        data_pec = read_buffer[7+(8*current_ic)] | (read_buffer[6+(8*current_ic)]<<8);
        if(calc_pec != data_pec )
        {
            ic[c_ic].sctrl.rx_pec_match = 1;
        }
        else ic[c_ic].sctrl.rx_pec_match = 0;

    }
    return(pec_error);
}

/*
Start Sctrl data communication
This command will start the sctrl pulse communication over the spins
*/
void LTC681x_stsctrl()
{
	uint8_t cmd[4];
    uint16_t cmd_pec;

    cmd[0] = 0x00;
    cmd[1] = 0x19;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    cs_low();
    spi_write_array(4,cmd);
    cs_high();
}



/*
The command clears the Sctrl registers and initializes
all values to 0. The register will read back hexadecimal 0x00
after the command is sent.
*/




/* Helper function to control discharge time value */
void LTC681x_set_cfgr_dcto(uint8_t nIC, cell_asic *ic,bool dcto[])
{
	for(int i =0;i<4;i++)
	{
		if(dcto[i])ic[nIC].config.tx_data[5] = ic[nIC].config.tx_data[5]|(0x01<<(i+4));
		else ic[nIC].config.tx_data[5] = ic[nIC].config.tx_data[5]&(~(0x01<<(i+4)));
	}
}

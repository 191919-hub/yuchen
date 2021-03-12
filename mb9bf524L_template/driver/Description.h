/*
        *** THIS IS A FILE FOR DOXYGEN AND NOT A C HEADER FILE ***
*/

/**
 *******************************************************************************
 ** \mainpage FM Peripheral Driver Library - Documentation
 ** <hr>
 *******************************************************************************
 ** \image html cypress_logo.png
 **
 ** Copyright (C) 2013-2015, Cypress Semiconductor Corporation. All rights reserved.    
 **                                                                                     
 ** This software, including source code, documentation and related                     
 ** materials ( "Software" ), is owned by Cypress Semiconductor                         
 ** Corporation ( "Cypress" ) and is protected by and subject to worldwide              
 ** patent protection (United States and foreign), United States copyright              
 ** laws and international treaty provisions. Therefore, you may use this               
 ** Software only as provided in the license agreement accompanying the                 
 ** software package from which you obtained this Software ( "EULA" ).                  
 ** If no EULA applies, Cypress hereby grants you a personal, nonexclusive,             
 ** non-transferable license to copy, modify, and compile the                           
 ** Software source code solely for use in connection with Cypress's                    
 ** integrated circuit products. Any reproduction, modification, translation,           
 ** compilation, or representation of this Software except as specified                 
 ** above is prohibited without the express written permission of Cypress.              
 ** Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO                                
 ** WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,                                
 ** BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED                                        
 ** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A                                     
 ** PARTICULAR PURPOSE. Cypress reserves the right to make                              
 ** changes to the Software without notice. Cypress does not assume any                 
 ** liability arising out of the application or use of the Software or any              
 ** product or circuit described in the Software. Cypress does not                      
 ** authorize its products for use in any products where a malfunction or               
 ** failure of the Cypress product may reasonably be expected to result in              
 ** significant property damage, injury or death ( "High Risk Product" ). By            
 ** including Cypress's product in a High Risk Product, the manufacturer                
 ** of such system or application assumes all risk of such use and in doing             
 ** so agrees to indemnify Cypress against all liability. 
 **
 ** <br>
 ** <hr>
 ** <b>History</b>
 **  - 2014-12-11
 **    - Version 0.1
 **    - Author: Spansion Inc., China
 **    - Comment: First version
 **
 **  - 2014-12-19
 **    - Version 0.2
 **    - Author: Spansion Inc., China
 **    - Comment:
 **      - Add RTC, VBAT, LPM driver
 **      - Add examples of WDG, DT, RTC, MFT
 **      - Support S6E2CDHx series
 **
 **  - 2015-01-22
 **    - Version 0.3
 **    - Author: Spansion Inc., China
 **    - Comment:
 **      - Add I2S-Lite, VBAT, Reset, LVD drivers
 **      - Add I2S-Lite, VBAT, LPM, LVD examples
 **      - Support S6E100xF series
 **
 **  - 2015-02-13
 **    - Version 0.4
 **    - Author: Spansion Inc., China
 **    - Comment:
 **      - Add Program-CRC, DSTC, LCD, I2S, HyperBus, Flash, External Bus I/F drivers and examples.
 **      - Add utility drivers for HyperFlash, Nand Flash, SDRAM, PSRAM and Seg-LCD.
 **      - Support S6E1B8X/S6E1B7X.
 **
 **  - 2015-03-30
 **    - Version 0.5
 **    - Author: Spansion Inc., China
 **    - Comment:
 **      - Add HSSPI, SDIF, CAN, CANFD, HDMI-CEC driver and examples
 **
 **  - 2015-04-01
 **    - Version 1.0
 **    - Author: Spansion Inc., China
 **    - Comment:
 **      - First formal release
 **
  **  - 2015-08-17
 **    - Version 1.1.0
 **    - Author: Cypress Inc., China
 **    - Comment:
 **      - Support S6E2GMx series 
 **
 **  - 2015-09-22
 **    - Version 1.1.0
 **    - Author: Cypress Inc., China
 **    - Comment:
 **      - Support S6E2HGx, S6E1C3x series 
 **
 **  - 2015-09-29
 **    - Version 2.0.0
 **    - Author: Cypress Inc., China
 **    - Comment:
 **      - Change location of pdl_device.h  
 **
 ** <hr>
 **
 ** <b>Driver:</b>
 ** <table border="0">
 ** <tr><td> ADC   </td><td>&nbsp;Analog Digital Cionverter                              </td></tr>
 ** <tr><td> BT    </td><td>&nbsp;Base Time                                              </td></tr>
 ** <tr><td> CAN   </td><td>&nbsp;Controller Area Network                                </td></tr>
 ** <tr><td> CANFD </td><td>&nbsp;CAN with Flexible Data Rate                            </td></tr>
 ** <tr><td> CLK   </td><td>&nbsp;Clock Module                                           </td></tr>
 ** <tr><td> CRC   </td><td>&nbsp;Cyclic Redundancy Check                                </td></tr>
 ** <tr><td> CR    </td><td>&nbsp;High-CR trimming                                       </td></tr>
 ** <tr><td> CSV   </td><td>&nbsp;Clock supervisor                                       </td></tr>
 ** <tr><td> DAC   </td><td>&nbsp;Digital Analog Converter                               </td></tr>
 ** <tr><td> DMA   </td><td>&nbsp;Direc Memory Access                                    </td></tr>
 ** <tr><td> DT    </td><td>&nbsp;Dual Timer                                             </td></tr>
 ** <tr><td> DTSC  </td><td>&nbsp;Descriptor System data Transfer Controller             </td></tr>
 ** <tr><td> EXINT </td><td>&nbsp;External Interrupt                                     </td></tr>
 ** <tr><td> EXTIF </td><td>&nbsp;External Bus Inerface                                  </td></tr>
 ** <tr><td> Flash </td><td>&nbsp;Main/Work Flash operation                              </td></tr>
 ** <tr><td> GPIO  </td><td>&nbsp;General Purpose Inoput/Output Ports (only header file) </td></tr>
 ** <tr><td> HBIF  </td><td>&nbsp;Hyper Bus Interface                                    </td></tr>
 ** <tr><td> HSSPI </td><td>&nbsp;High Speed Quad SPI                                    </td></tr>
 ** <tr><td> I2CS  </td><td>&nbsp;I2C Slave                                              </td></tr>
 ** <tr><td> I2S   </td><td>&nbsp;Inter-IC Sound                                         </td></tr>
 ** <tr><td> I2SL  </td><td>&nbsp;I2S Lite Version                                       </td></tr>
 ** <tr><td> ICC   </td><td>&nbsp;IC Card Interface                                      </td></tr>
 ** <tr><td> LCD   </td><td>&nbsp;Segment LCD Controller                                 </td></tr>
 ** <tr><td> LPM   </td><td>&nbsp;Low Power Modes                                        </td></tr>
 ** <tr><td> LVD   </td><td>&nbsp;Low Voltage Detection                                  </td></tr>
 ** <tr><td> MFS   </td><td>&nbsp;Multi Function Serial Interface                        </td></tr>
 ** <tr><td> MFT   </td><td>&nbsp;Multi Function Timer                                   </td></tr>
 ** <tr><td> PCRC  </td><td>&nbsp;Programmable Cyclic Redundancy Check                   </td></tr>
 ** <tr><td> PPG   </td><td>&nbsp;Programmable Pulse Generator                           </td></tr>
 ** <tr><td> QPRC  </td><td>&nbsp;Quad Decoder and Position/Revolution Counter           </td></tr>
 ** <tr><td> RC    </td><td>&nbsp;HDMI-CEC/Remote Control Reception/Transmission         </td></tr>
 ** <tr><td> RESET </td><td>&nbsp;Reset Cause Register                                   </td></tr>
 ** <tr><td> RTC   </td><td>&nbsp;Real Time Clock                                        </td></tr>
 ** <tr><td> SDIF  </td><td>&nbsp;SD Card Interface                                      </td></tr>
 ** <tr><td> UID   </td><td>&nbsp;Unique ID                                              </td></tr>
 ** <tr><td> USB   </td><td>&nbsp;Universal Serial Bus                                   </td></tr>
 ** <tr><td> VBAT  </td><td>&nbsp;VBAT Domain                                            </td></tr>
 ** <tr><td> WC    </td><td>&nbsp;Watch Counter                                          </td></tr>
 ** <tr><td> WDG   </td><td>&nbsp;Watch Dog Timer (Hardware/Software)                    </td></tr>
 ** </table>
 **
 ** <b>Utility:</b>
 ** <table border="0">
 ** <tr><td> EEPROM         </td><td> I2C interface EEPROM                          </td></tr>
 ** <tr><td> HYPER_FLASH    </td><td> Hyper Bus Flash                               </td></tr>
 ** <tr><td> I2S_CODEC      </td><td> I2S codec                                     </td></tr>
 ** <tr><td> NAND_FLASH     </td><td> Access Nand Flash via External Bus Interface  </td></tr>
 ** <tr><td> PRINTF         </td><td> Printf via UART                               </td></tr>
 ** <tr><td> PSRAM          </td><td> Access PSRAM via External Bus Interface       </td></tr>
 ** <tr><td> QSPI_FLASH     </td><td> Access Quad-SPI Flash via High speed Quad-SPI </td></tr>
 ** <tr><td> SD_CARD        </td><td> Access SD card via SDIO interface             </td></tr>
 ** <tr><td> SDRAM          </td><td> Access SDRAM via External Bus Interface       </td></tr>
 ** <tr><td> SEG_LCD        </td><td> Control Segment LCD via LCD controller        </td></tr>
 ** </table>
 **
** <b>Examples:</b>
 **
 ** - ADC
 **   - <pre>adc_scan_multich_irq_bt       ADC multi-channels scan sample with interrupt mode triggered by base timer</pre>
 **   - <pre>adc_scan_multich_irq_sw       ADC multi-channels scan conversion with usage of interrupt callback triggered by software</pre>
 **   - <pre>adc_scan_multich_polling_sw   ADC multi-channels scan conversion with polling for conversion finish triggered by software</pre>
 **   - <pre>adc_scan_onech_irq_sw         ADC single-channel scan sample with interrupt mode triggered by software</pre>
 **   - <pre>adc_scan_onech_polling_sw     ADC single-channel scan conversion with polling for conversion finish triggered by software</pre>
 **   - <pre>adc_scan_dma                  ADC single-channel scan sample using DMA transfer</pre>
 **   - <pre>adc_prio_irq_sw               ADC priority conversion with usage of interrupt callback triggered by software</pre>
 **   - <pre>adc_prio_polling_sw           ADC priority conversion with polling for finished conversion triggered by software</pre>
 **   - <pre>adc_range_compare             ADC result range comparison function implementation</pre>
 **   - <pre>adc_scan_onech_mft_frt        ADC single-channel scan sample with interrupt mode triggered by Free Run Timer of MFT</pre>
 **   - <pre>adc_scan_onech_mft_adcmp      ADC single-channel scan sample with interrupt mode triggered by ADC compare unit of MFT</pre>
 **
 ** - BT
 **   - <pre>bt_iomode        Base Timer IO modes demonstration</pre>
 **   - <pre>bt_ppg           Base Timer configured as Programmable Pulse Generator</pre>
 **   - <pre>bt_pwc           Base Timer configured as Pulse Width Counter</pre>
 **   - <pre>bt_pwm           Base Timer configured as Pulse With Modulator</pre>
 **   - <pre>bt_rt            Base Timer configured as Reload Timer </pre>
 **
 ** - CAN
 **   - <pre>can_simple       Perform CAN standard frame transmission and reception</pre>
 **
 ** - CANFD
 **   - <pre>canfd_simple     Perform CANFD standard frame transmission and reception</pre>
 **
 ** - CLK
 **   - <pre>clk_gating       Clock gating setting</pre>
 **   - <pre>clk_init         User clock initialization</pre>
 **
 ** - CR
 **   - cr_trimming
 **     - fm0p
 **       - <pre>type1_2      The CR-trimming samples of FM0P type 1 and 2 products</pre>
 **     - fm3
 **       - <pre>type0        The CR-trimming samples of FM3 type 0 products</pre>
 **       - <pre>type1_5      The CR-trimming samples of FM3 type 1 and 5 products</pre>
 **       - <pre>type2_4      The CR-trimming samples of FM3 type 2 and 4 products</pre>
 **       - <pre>type3_7      The CR-trimming samples of FM3 type 3 and 7 products</pre>
 **       - <pre>type8_9_10_11_12  The CR-trimming samples of FM3 type 8-12 products</pre>
 **     - fm4
 **       - <pre>type1_2_3_4  The CR-trimming samples of FM4 type 1,2,3,4 products</pre>
 **
 ** - CRC
 **   - <pre>crc_16_32        Calculation of CRC-16 and CRC-32</pre>
 **
 ** - CSV
 **   - <pre>csv_reset        CSV reset demonstration when main/sub clock frequency is abnormal</pre>
 **   - <pre>fcs_int          FCS interrupt demonstration when main clock frequency is abnormal</pre>
 **   - <pre>fcs_reset        CSV reset demonstration when main clock frequency is abnormal</pre>
 **
 ** - DAC
 **   - <pre>dac_sin_tri_wave Output sin and triangle wave via DAC</pre>
 **
 ** - DMA
 **   - <pre>dma_software     The DMA perform a sotware block transfer (For demand transfer see ADC and MFS)</pre>
 **
 ** - DSTC
 **   - <pre>dstc_sw_transfer Transfer data using DSTC triggered by software</pre>
 **   - <pre>dstc_hw_transfer Transfer data using DSTC triggered by Base Timer 0</pre>
 **
 ** - DT
 **   - <pre>dt_unuse_int     Dual Timer configured as periodic mode for ch.0 and one-shot mode for ch.1 with usage interruption callback</pre>
 **   - <pre>dt_use_int       Dual Timer configured as periodic mode for ch.0 and one-shot mode for ch.1 with polling for interruption</pre>
 **
 ** - EXINT
 **   - <pre>exint_simple     Simple external interrupt occurence demonstration</pre>
 **   - <pre>exint_dma        Using external interrupt to trigger DMA transfer</pre>
 **
 ** - Flash
 **   - fm0p
 **     - <pre>type1_2   Implement Flash operation of FM0+ type 1 and 2 products</pre>
 **   - fm3
 **     - <pre>type0     Implement Flash operation of FM3 type 0 products</pre>
 **     - <pre>type1_2_5 Implement Flash operation of FM3 type 1,2,5 products</pre>
 **     - <pre>type3_7   Implement Flash operation of FM3 type 3,7 products</pre>
 **     - <pre>type6_8_9 Implement Flash operation of FM3 type 6,8,9 products</pre>
 **     - <pre>type10_11 Implement Flash operation of FM3 type 10,11 products</pre>
 **     - <pre>type12    Implement Flash operation of FM3 type 12 products</pre>
 **   - fm4
 **     - <pre>type1     Implement Flash operation of FM4 type 1 products</pre>
 **     - <pre>type2_3_4 Implement Flash operation of FM4 type 2,3,4 products</pre>
 **
 ** - GPIO
 **   - <pre>gpio_set        Simple GPIO pin setting</pre>
 **   - <pre>fgpio_set       Simple Fast GPIO pin setting</pre>
 **
 ** - HBIF
 **   - <pre>hbif_access_s26kl512s     Access Hyper Flash S26KL512S via HyperBus interface</pre>
 **
 ** - HSSPI
 **   - <pre>hsspi_access_s25l164k  Access Flash S25L164K via High Speed Quad SPI interface</pre>
 **
 ** - I2CS
 **   - i2c_slave_wkup
 **     - <pre>wkup_master  Send correponding slave address or reserved address to wakeup slave MCU from standby mode via i2S Slave I/F</pre>
 **     - <pre>wkup_slave   Enter standby mode after initializing I2C Slave and wait for wakeup via I2C slave</pre>
 **
 ** - I2S
 **   - <pre>i2s_play_wave_via_wm8731  Play wave files by I2S via I2S codec WM8731</pre>
 **
 ** - I2SL
 **   - <pre>i2s_irq_play_wave_via_wm8731       Play wave data via WM8731 using I2S-Lite with interrupt callback</pre>
 **   - <pre>i2s_polling_play_wave_via_wm8731   Play wave data via WM8731 using I2S-Lite with interrupt flag polling</pre>
 **
 ** - ICC
 **   - <pre>icc_atr_interrupt  Receive ATR from IC card via IC card interface with interrupt enabled</pre>
 **
 ** - LCD
 **   - <pre>cl1010   Use LCD controller to control Segment LCD CL1010</pre>
 **   - <pre>tsdh1188 Use LCD controller to control Segment LCD TSDH1188</pre>
 **
 ** - LPM
 **   - <pre>lpm_sleep       MCU is set to sleep mode, EXINT3 wakes up MCU</pre>
 **   - <pre>lpm_rtc_mode    MCU is set to RTC mode, RTC timer interrupt wakes up MCU</pre>
 **   - <pre>lpm_deep_standby_rtc_mode  Deep standby RTC mode waken up by RTC interrupt</pre>
 **
 ** - LVD
 **   - lvd_interrupt
 **     - <pre>fm0p_type2      Detect low voltage by generating interrupt on TYPE2 product of FM0+</pre>
 **     - <pre>fmx_other       Detect low voltage by generating interrupt on other products</pre>
 **   - lvd_polling
 **     - <pre>fm0p_type2      Detect low votage by polling interrupt flag on TYPE2 product of FM0+</pre>
 **     - <pre>fmx_other       Detect low votage by polling interrupt flag on other products</pre>
 **
 ** - MFS
 **   - CSIO
 **     - <pre>csio_interrupt      CSIO communication between 2 channels(one for Master, the other for Slave) with interrupt enabled</pre>
 **     - <pre>csio_polling        CSIO communication between 2 channels(one for Master, the other for Slave) with interrupt flag polling</pre>
 **     - <pre>csio_dma            CSIO communication between 2 channels(one for Master, the other for Slave) with interrupt enabled using DMA</pre>
 **     - <pre>csio_fifo_interrupt CSIO communication between 2 channels(one for Master, the other for Slave) with interrupt enabled using FIFO</pre>
 **     - <pre>csio_trig_by_sync_timer CSIO communication between 2 channels(one for Master, the other for Slave) with interrupt enabled triggered by Serial timer</pre>
 **     - <pre>csio_using_cs           CSIO communication between 2 channels(one for Master, the other for Slave) with interrupt enabled and CS pin enabled </pre>
 **     - <pre>csio_flash              Using CSIO to access to SPI Flash</pre>
 **   - I2C
 **     - <pre>i2c_polling</pre>
 **       - <pre>i2c_master_polling   I2C channel acts as I2C master with TX/RX completion polling, which communicates with the slave "i2c_slave_polling"</pre>
 **       - <pre>i2c_slave_polling    I2C channel acts as I2C slave with TX/RX completion polling, which communicates with the slave "i2c_master_polling"</pre>
 **     - <pre>i2c_eeprom             Access to EEPROm with I2C interface with interrupt mode or polling mode</pre>
 **   - LIN
 **     - <pre>lin_interrupt</pre>
 **       - <pre>lin_master           LIN channel acts as LIN master with interrupt mode, which communicates with the slave "lin_slave" at current folder</pre>
 **       - <pre>lin_slave            LIN channel acts as LIN slave with interrupt mode, which communicates with the master "lin_master" at current folder</pre>
 **     - <pre>lin_polling</pre>
 **       - <pre>lin_master           LIN channel acts as LIN master with interrupt polling mode, which communicates with the slave "lin_slave" at current folder</pre>
 **       - <pre>lin_slave            LIN channel acts as LIN slave with interrupt polling mode, which communicates with the master "lin_master" at current folder</pre>
 **   - UART
 **     - <pre>uart_interrupt      UART communication between 2 channels with interrupt enabled</pre>
 **     - <pre>uart_polling        UART communication between 2 channels with interrupt flag polling</pre>
 **     - <pre>uart_dma            UART communication between 2 channels with interrupt enabled using DMA</pre>
 **     - <pre>uart_fifo_interrupt UART communication between 2 channels with interrupt enabled using FIFO</pre>
 **
 ** - MFT
 **   - frt
 **     - <pre>frt_int                  Free-run Timer function with usage of zero/peak match interrupt callback</pre>
 **     - <pre>frt_polling              Free-run Timer function with polling for zero/peak match</pre>
 **   - icu
 **     - <pre>icu_int                  Input Captuer Unit function with usage of interrupt callback</pre>
 **     - <pre>icu_polling              Input Captuer Unit function with polling for capture completion</pre>
 **   - ocu
 **     - fm3_fm4_fm0
 **       - <pre>ocu_ch_linked            Perform 2 change linked mode of Output Compare Unit with FM3 compatile mode</pre>
 **       - <pre>ocu_ch_single            Perform 1-change/low-active/high-active mode of Output Compare Unit with FM3 compatile mode</pre>
 **     - fm4_fm0
 **       - <pre>ocu_ch_linked            Perform 2 change linked mode of Output Compare Unit</pre>
 **       - <pre>ocu_ch_single            Perform 1-change/low-active/high-active mode of Output Compare Unit</pre>
 **   - wfg
 **     - fm3_fm0_fm4
 **       - <pre>wfg_nzcl_dtif_dttix_trig Perform DTIF interrupt triggered DTTIX with FM3 compatile mode</pre>
 **       - <pre>wfg_nzcl_dtif_sw_trig    Perform DTIF interrupt triggered software with FM3 compatile mode</pre>
 **       - <pre>wfg_ppg_dead_timer_mode  Perform PPG-dead timer mode of Waveform Generator Unit with FM3 compatile mode</pre>
 **       - <pre>wfg_rt_dead_timer_mode   Perform RT-dead timer mode of Waveform Generator Unit with FM3 compatile mode</pre>
 **       - <pre>wfg_rt_ppg_mode          Perform RT-PPG mode of Waveform Generator Unit with FM3 compatile mode</pre>
 **       - <pre>wfg_through_mode         Perform through mode of Waveform Generator Unit with FM3 compatile mode</pre>
 **       - <pre>wfg_timer_ppg_mode       Perform Timer-PPG mode of Waveform Generator Unit with FM3 compatile mode</pre>
 **     - fm0_fm4
 **       - <pre>wfg_nzcl_dtif_dttix_trig Perform DTIF interrupt triggered DTTIX</pre>
 **       - <pre>wfg_nzcl_dtif_sw_trig    Perform DTIF interrupt triggered software</pre>
 **       - <pre>wfg_ppg_dead_timer_mode  Perform PPG-dead timer mode of Waveform Generator Unit with</pre>
 **       - <pre>wfg_rt_dead_timer_mode   Perform RT-dead timer mode of Waveform Generator Unit</pre>
 **       - <pre>wfg_rt_ppg_mode          Perform RT-PPG mode of Waveform Generator Unit with</pre>
 **       - <pre>wfg_through_mode         Perform through mode of Waveform Generator Unit with</pre>
 **       - <pre>wfg_timer_ppg_mode       Perform Timer-PPG mode of Waveform Generator Unit with</pre>
 **
 ** - PCRC
 **   - <pre>pcrc_gen  Generate CRC code with programmable CRC</pre>
 **
 ** - QPRC
 **   - <pre>qprc_pc_directional        Perform Count mode with direction (mode 3) of Position Counter</pre>
 **   - <pre>qprc_pc_updown             Perform Up/down count mode (mode 1) of Position Counter</pre>
 **   - <pre>qprc_pc_phase_diff         Perform Phase different count mode (mode 2) of Position Counter</pre>
 **   - <pre>qprc_rc_pc_ofuf_trig       Perform Revolution Counter mode 2 (triggered by Position Counter)</pre>
 **   - <pre>qprc_rc_zin_pcofuf_trig    Perform Revolution Counter mode 3 (triggered by ZIN or Position Counter)</pre>
 **   - <pre>qprc_rc_zin_trig           Perform Revolution Counter mode 1 (triggered by ZIN)</pre>
 **
 ** - RC
 **   - cec
 **     - <pre>cec_rx_interrupt  Perform CEC format data reception using interrupt callback</pre>
 **     - <pre>cec_rx_polling    Perform CEC format data reception with interrupt flag polling</pre>
 **     - <pre>cec_tx_interrupt  Perform CEC format data tranmission using interrupt callback</pre>
 **     - <pre>cec_tx_polling    Perform CEC format data tranmission with interrupt flag polling</pre>
 **   - nec
 **     - <pre>nec_rx_interrupt  Perform NEC format data reception using interrupt callback</pre>
 **     - <pre>nec_rx_polling    Perform NEC format data reception with interrupt flag polling</pre>
 **
 ** - RTC
 **   - <pre>rtc_count        Usage of time counting and alarm interruption for RTC</pre>
 **   - <pre>rtc_timer        Usage of timer interruption for RTC</pre>
 **   - <pre>rtc_calibration  Usage of calibration function for RTC  </pre>
 **
 ** - SDIF
 **   - <pre>sd_card_block_rw  Perform data block read/write from/to SD memory card via SDIO inferface</pre>
 **
 ** - UID
 **   - <pre>uid_read          Several methods to read the Unique ID</pre>
 **
 ** - WC
 **   - <pre>wc_int            Watch Counter function with usage of interrupt callback</pre>
 **   - <pre>wc_polling        Watch Counter function with polling for interrupt occurrence</pre>
 **
 ** - WDG
 **   - <pre>hwwdg             Usage of hardware watchdog</pre>
 **   - <pre>swwdg</pre>
 **     - <pre>swwdg_normal    Usage of normal mode for software watchdog</pre>
 **     - <pre>swwdg_window    Usage of window mode for software watchdog</pre>
 **
 **
 ** <b>Usage of example code:</b>
 **
 ** Each example code consists of a 'main.c' and a 'pdl_user.h' module.
 ** The main module contains the example code, where 'pdl_user.h'
 ** contains all necessary settings and definitions for the example
 ** itself.
 ** To use and compile an example, just copy these two modules
 ** to the 'template/source' folder, overwriting the modules located
 ** there. The overwritten modules have backups in the 'template/
 ** source/backup' folder for recovery.
 ** The default device selected is MB9BF506R, if other device is used,
 ** refer to "fm_pdl_quick_start_guide.pdf" about how to adjust the setting
 ** in the IAR and Keil for this device.
 ** Follow the explanations and process in the main.c module's
 ** description in the comment lines at the top.
 **
 **
 ** <br><hr><br>
 **
 ******************************************************************************/



#ifndef TYPEDEFINE_H
#define TYPEDEFINE_H


typedef union 
{
  volatile unsigned char Byte;
  struct {
   volatile  unsigned char b0        :1;                                       /* Bit 0 */
   volatile  unsigned char b1        :1;                                       /* Bit 1 */
   volatile  unsigned char b2        :1;                                       /* Bit 2 */
   volatile  unsigned char b3        :1;                                       /* Bit 3 */
   volatile  unsigned char b4        :1;                                       /* Bit 4 */
   volatile  unsigned char b5        :1;                                       /* Bit 5 */
   volatile  unsigned char b6        :1;                                       /* Bit 6 */
   volatile  unsigned char b7        :1;                                       /* Bit 7 */
  } Bits;
} BitType;









#endif
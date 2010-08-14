#ifndef _BITREV_H
#define _BITREV_H
static inline unsigned char bitrev8(unsigned char val)
{
        unsigned h = 0;
        int i;
        // loop through all the bits
        for(i = 0; i<8; i++)
        {
                // add bit from value to 1 bit left shifted variable
                h = (h << 1) + (val & 1);
                // right shift bits by 1
                val >>= 1;
        }
        return h;
}
#endif /* _BITREV_H */

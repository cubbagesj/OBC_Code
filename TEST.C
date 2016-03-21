  for(int j=0;j<32;j++)
    {
    pcm[3*j]   = ~lookup[(data2[2*j] >> 4) & 0x00ff];
    pcm[3*j+1] = ~lookup[((data2[2*j]<<4) & 0x00f0) | ((data2[2*j+1]>>8)&0x000f)];
    pcm[3*j+2] = ~lookup[(data2[2*j+1] & 0x00ff)];
    }

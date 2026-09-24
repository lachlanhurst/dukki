// unpack10 in.raw width height bytes_per_line out.raw16 [black]  -- Rockchip CIF RG10: 40-bit little-endian stream, 10 bits per pixel,
// four pixels per five bytes, row padded to bytes_per_line. Output 16-bit LE with black subtracted and scaled x64. Prints channel means.
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
int main(int argc,char**argv){
  if(argc<6){fprintf(stderr,"usage\n");return 1;}
  int w=atoi(argv[2]),h=atoi(argv[3]),bpl=atoi(argv[4]); int black=argc>6?atoi(argv[6]):64;
  FILE*in=fopen(argv[1],"rb"); uint8_t*row=malloc(bpl); uint16_t*out=malloc((size_t)w*2); uint16_t*raw=malloc((size_t)w*2);
  FILE*o=fopen(argv[5],"wb"); double sum[4]={0}; long cnt[4]={0}; long sat=0;
  for(int y=0;y<h;y++){ if(fread(row,1,bpl,in)!=(size_t)bpl){fprintf(stderr,"short read\n");return 1;}
    for(int x=0,i=0;x<w;x+=4,i+=5){uint64_t v=row[i]|((uint64_t)row[i+1]<<8)|((uint64_t)row[i+2]<<16)|((uint64_t)row[i+3]<<24)|((uint64_t)row[i+4]<<32);
      raw[x]=v&1023; raw[x+1]=(v>>10)&1023; raw[x+2]=(v>>20)&1023; raw[x+3]=(v>>30)&1023;}
    for(int x=0;x<w;x++){int c=((y&1)<<1)|(x&1); sum[c]+=raw[x]; cnt[c]++; if(raw[x]>=1020)sat++; int v=(raw[x]-black)*64; out[x]=v<0?0:v>65535?65535:v;}
    fwrite(out,2,w,o);}
  fclose(o); printf("channel means R %.1f Gr %.1f Gb %.1f B %.1f (10-bit), saturated %.2f%%\n",sum[0]/cnt[0],sum[1]/cnt[1],sum[2]/cnt[2],sum[3]/cnt[3],100.0*sat/((double)w*h));
  return 0;}

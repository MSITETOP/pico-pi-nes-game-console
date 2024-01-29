#include "InfoNES_System.h"
#include "InfoNES.h"
#include <string.h>
#include <stdio.h>

#include "rom.c"

#include "f_util.h"
#include "ff.h"
#include "hw_config.h"
//#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"

#include "ff_headers.h"
#include "ff_stdio.h"

//NES_DISP_WIDTH  256
//NES_DISP_HEIGHT 240

WORD NesPalette[64]={
  0x738E,0x88C4,0xA800,0x9808,0x7011,0x1015,0x0014,0x004F,
  0x0148,0x0200,0x0280,0x11C0,0x59C3,0x0000,0x0000,0x0000,
  0xBDD7,0xEB80,0xE9C4,0xF010,0xB817,0x581C,0x015B,0x0A59,
  0x0391,0x0480,0x0540,0x3C80,0x8C00,0x0000,0x0000,0x0000,
  0xFFDF,0xFDC7,0xFC8B,0xFC48,0xFBDE,0xB39F,0x639F,0x3CDF,
  0x3DDE,0x1690,0x4EC9,0x9FCB,0xDF40,0x0000,0x0000,0x0000,
  0xFFDF,0xFF15,0xFE98,0xFE5A,0xFE1F,0xDE1F,0xB5DF,0xAEDF,
  0xA71F,0xA7DC,0xBF95,0xCFD6,0xF7D3,0x0000,0x0000,0x0000,
};

/* Menu screen */
int InfoNES_Menu()
{
    return 0;
}


#if 0 
// 直接读入写入
/* Read ROM image file */
int InfoNES_ReadRom( const char *pszFileName )
{
/*
 *  Read ROM image file
 *
 *  Parameters
 *    const char *pszFileName          (Read)
 *
 *  Return values
 *     0 : Normally
 *    -1 : Error
 */

  FF_FILE *pxFile;
  pxFile = ff_fopen(filename, "r");
  if (!pxFile)
      printf("fopen(%s): %s (%d)\n", "/roms.txt", strerror(errno), -errno);
  assert(pxFile);

  ff_fseek(pxFile, 0, FF_SEEK_END);
  long file_size = ff_ftell(pxFile) + 1;
  printf("File size: %d\n", file_size);
  //ff_fseek(pxFile, 0, FF_SEEK_SET);
  ff_fclose(pxFile);

  printf("fclose ok!\n");

  sleep_ms(10);
  pxFile = ff_fopen(filename, "r");
  if (!pxFile)
      printf("fopen(%s): %s (%d)\n", "/roms.txt", strerror(errno), -errno);
  assert(pxFile);

  //unsigned char nes_rom[file_size];
  unsigned char *nes_game_rom = (unsigned char *)malloc(file_size);
  unsigned char buff[ 64 ];
  long rom_offset = 0;
  size_t read_file_count;
  size_t i = 0;
  while (true)
  {
      read_file_count = ff_fread(buff, 64, 1, pxFile);
      //printf("read_file_count: %d \n", read_file_count);
      if (read_file_count <= 0 )
      {
        //printf("if (read_file_count < 0 )\n");
        break;
      }
      else
      {
          for(i = 0; i < read_file_count; i++)
          {
              nes_game_rom[rom_offset] = buff[i];
              rom_offset++;
          }
      }
  }
  
  /* Close the file. */
  ff_fclose(pxFile);

  printf("Initialized. ROM@%p\n", nes_game_rom);
  printf("HEAP: (%x %x %x %x)\n", nes_game_rom[0], nes_game_rom[1], nes_game_rom[2], nes_game_rom[4]);

  /* Read ROM Header */
  //unsigned char * rom = (unsigned char*)nes_rom;
  unsigned char * rom = nes_game_rom;

  //printf("nes rom size: %d\n", sizeof(nes_rom));
  memcpy( &NesHeader, rom, sizeof(NesHeader));
  if ( memcmp( NesHeader.byID, "NES\x1a", 4 ) != 0 )
  {
    /* not .nes file */
    printf("not .nes file\n");
    return -1;
  }
  rom += sizeof(NesHeader);

  /* Clear SRAM */
  memset( SRAM, 0, SRAM_SIZE );

  /* If trainer presents Read Triner at 0x7000-0x71ff */
  if ( NesHeader.byInfo1 & 4 )
  {
    //memcpy( &SRAM[ 0x1000 ], rom, 512);
	rom += 512;
  }

  /* Allocate Memory for ROM Image */
  ROM = rom;
  rom += NesHeader.byRomSize * 0x4000;

  if ( NesHeader.byVRomSize > 0 )
  {
    /* Allocate Memory for VROM Image */
	VROM = (unsigned char*)rom;
	rom += NesHeader.byVRomSize * 0x2000;
  }

  printf("return 0;!\n");
  /* Successful */
  return 0;
}
#endif

const char * filename = "/nes/maliao.nes"; //"daodantanke.nes";  // 大于45kb不行？
//const char * filename = "/nes/zhadanren1.nes"; //"daodantanke.nes";  // 大于45kb不行？
//extern const unsigned char nes_rom[];
int InfoNES_ReadRom( const char *pszFileName )
{
/*
 *  Read ROM image file
 *
 *  Parameters
 *    const char *pszFileName          (Read)
 *
 *  Return values
 *     0 : Normally
 *    -1 : Error
 */
  FF_FILE *pxFile;
  pxFile = ff_fopen(filename, "r");
  if (!pxFile)
      printf("fopen(%s): %s (%d)\n", "/roms.txt", strerror(errno), -errno);
  assert(pxFile);
  ff_fread(&NesHeader, sizeof NesHeader, 1, pxFile);

  if ( memcmp( NesHeader.byID, "NES\x1a", 4 ) != 0 )
  {
    /* not .nes file */
    printf("not .nes file\n");
    return -1;
  }

  /* Clear SRAM */
  memset( SRAM, 0, SRAM_SIZE );

  /* If trainer presents Read Triner at 0x7000-0x71ff */
  if ( NesHeader.byInfo1 & 4 )
  {
    ff_fread( &SRAM[ 0x1000 ], 512, 1, pxFile );
  }

  /* Allocate Memory for ROM Image */
  ROM = (BYTE *)malloc( NesHeader.byRomSize * 0x4000 );

  /* Read ROM Image */
  ff_fread( ROM, 0x4000, NesHeader.byRomSize, pxFile );

  if ( NesHeader.byVRomSize > 0 )
  {
    /* Allocate Memory for VROM Image */
    VROM = (BYTE *)malloc( NesHeader.byVRomSize * 0x2000 );

    /* Read VROM Image */
    ff_fread( VROM, 0x2000, NesHeader.byVRomSize, pxFile );
  }

  ff_fclose(pxFile);

  printf("Read file successful!!!\n");
  /* Successful */
  return 0;
}

/* Release a memory for ROM */
void InfoNES_ReleaseRom()
{
}

extern void tft_flush();
/* Transfer the contents of work frame on the screen */
void InfoNES_LoadFrame()
{
    /* WorkFrame: 256 x 240 */
    tft_flush();

}


extern int infones_100ask_get_pdwPad_state(void);

/* Get a joypad state */
void InfoNES_PadState( DWORD *pdwPad1, DWORD *pdwPad2, DWORD *pdwSystem )
{
    /*
  *  Get a joypad state
  *
  *  Parameters
  *    DWORD *pdwPad1                   (Write)
  *      Joypad 1 State
  *
  *    DWORD *pdwPad2                   (Write)
  *      Joypad 2 State
  *
  *    DWORD *pdwSystem                 (Write)
  *      Input for InfoNES
  *
*/
    *pdwPad1   = (DWORD)infones_100ask_get_pdwPad_state();
    //*pdwSystem = dwKeySystem;
    *pdwPad2   = 0;
    *pdwSystem = 0;
    //printf("dwKeyPad1: %d\n", *pdwPad1);
}


/* memcpy */
void *InfoNES_MemoryCopy( void *dest, const void *src, int count )
{
    return memcpy(dest,src,count);
}


/* memset */
void *InfoNES_MemorySet( void *dest, int c, int count )
{
    return memset(dest,c,count);
}


/* Print debug message */
void InfoNES_DebugPrint( char *pszMsg )
{
}


/* Wait */
void InfoNES_Wait()
{
}


/* Sound Initialize */
void InfoNES_SoundInit( void )
{
}


/* Sound Open */
int InfoNES_SoundOpen( int samples_per_sync, int sample_rate )
{
    return 0;
}


/* Sound Close */
void InfoNES_SoundClose( void )
{
}


/* Sound Output 5 Waves - 2 Pulse, 1 Triangle, 1 Noise, 1 DPCM */
void InfoNES_SoundOutput(int samples, unsigned char *wave1, unsigned char *wave2, unsigned char *wave3, unsigned char *wave4, unsigned char *wave5)
{
}


/* Print system message */
void InfoNES_MessageBox( char *pszMsg, ... )
{
}


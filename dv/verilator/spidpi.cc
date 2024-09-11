
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if SPIDPI_STANDALONE
typedef uint32_t svBitVecVal;
// For stricter compliation checks, and faster build checks.
#else
#include <svdpi.h>
#endif

#define logText printf

// TODO: Simply copied from lcd_std7735_cmds.h for now
typedef enum {
  ST7735_NOP     = 0x00,
  ST7735_SWRESET = 0x01,
  ST7735_RDDID   = 0x04,
  ST7735_RDDST   = 0x09,
  ST7735_SLPIN   = 0x10,
  ST7735_SLPOUT  = 0x11,
  ST7735_PTLON   = 0x12,
  ST7735_NORON   = 0x13,
  ST7735_INVOFF  = 0x20,
  ST7735_INVON   = 0x21,
  ST7735_DISPOFF = 0x28,
  ST7735_DISPON  = 0x29,
  ST7735_CASET   = 0x2A,
  ST7735_RASET   = 0x2B,
  ST7735_RAMWR   = 0x2C,
  ST7735_RAMRD   = 0x2E,
  ST7735_PTLAR   = 0x30,
  ST7735_COLMOD  = 0x3A,
  ST7735_MADCTL  = 0x36,
  ST7735_FRMCTR1 = 0xB1,
  ST7735_FRMCTR2 = 0xB2,
  ST7735_FRMCTR3 = 0xB3,
  ST7735_INVCTR  = 0xB4,
  ST7735_DISSET5 = 0xB6,
  ST7735_PWCTR1  = 0xC0,
  ST7735_PWCTR2  = 0xC1,
  ST7735_PWCTR3  = 0xC2,
  ST7735_PWCTR4  = 0xC3,
  ST7735_PWCTR5  = 0xC4,
  ST7735_VMCTR1  = 0xC5,
  ST7735_RDID1   = 0xDA,
  ST7735_RDID2   = 0xDB,
  ST7735_RDID3   = 0xDC,
  ST7735_RDID4   = 0xDD,
  ST7735_PWCTR6  = 0xFC,
  ST7735_GMCTRP1 = 0xE0,
  ST7735_GMCTRN1 = 0xE1,
} ST7735_Cmd;

typedef enum {
  ST77_MADCTL_MX  = 0x01 << 7,  // Column Address Order
  ST77_MADCTL_MV  = 0x01 << 6,  // Row/Column Exchange
  ST77_MADCTL_MY  = 0x01 << 5,  // Row Address Order
  ST77_MADCTL_ML  = 0x01 << 4,
  ST77_MADCTL_RGB = 0x01 << 3,
  ST77_MADCTL_MH  = 0x01 << 2
} ST77_MADCTL_Bits;

// Color definitions
typedef enum {
  ST7735_ColorBlack   = 0x0000,
  ST7735_ColorBlue    = 0x001F,
  ST7735_ColorRed     = 0xF800,
  ST7735_ColorGreen   = 0xE007,
  ST7735_ColorCyan    = 0x07FF,
  ST7735_ColorMagenta = 0xF81F,
  ST7735_ColorYellow  = 0xFFE0,
  ST7735_ColorWhite   = 0xFFFF,
} ST7735_Color;

class spidpi {
public:
  spidpi() {
    reset();
  }
  ~spidpi();
  
  void sckEdge(svBitVecVal in);

private:
  // TODO: we do not concern ourselves with orientation presently! tut!
  static constexpr unsigned width  = 160u;
  static constexpr unsigned height = 128u;
  typedef uint16_t pixel_t;

  void reset();

  void writePPM(const char *filename = nullptr);

  // Pixel buffer RAM.
  pixel_t frameBuf[height][width];

  bool pixWriting;  // Pixel writing mode?

  uint16_t colAdr;  // Current column.
  uint16_t rowAdr;  // Current row.

  uint16_t xStart;  // Start column.
  uint16_t xEnd;    // End column.
  uint16_t yStart;  // Start row.
  uint16_t yEnd;    // End row.
  pixel_t pixCol;   // Current pixel colour.

  uint8_t inByte;

  uint8_t nbits;
  uint8_t cmdLen;
  uint8_t cmd[0x20];
};

void spidpi::reset() {
  memset(frameBuf, 0u, sizeof(frameBuf));
  inByte = 0u;
  nbits = 0u;
  cmdLen = 0u;
  xEnd = xStart = 0u;
  yEnd = yStart = 0u;
  colAdr = 0u;
  rowAdr = 0u;
  pixWriting = false;
  writePPM();
}

void spidpi::writePPM(const char *filename) {
  if (!filename) filename = "lcd.ppm";
  FILE *out = fopen(filename, "wb");
  if (out) {
    fprintf(out, "P6 %u %u %u\n", width, height, 0x3fu);
    unsigned y = 0u;
    while (y < height) {
      const pixel_t *row = frameBuf[y];
      unsigned x = 0;
      while (x < width) {
        // Extract components from 5:6:5 format.
        // TODO: sort out the predictable R/B swap confusion.
#if 0
        unsigned b =  row[x] & 0x1fu;
        unsigned g = (row[x] >> 5) & 0x3fu;
        unsigned r =  row[x] >> 11;
#else
        unsigned r =  row[x] & 0x1fu;
        unsigned g = (row[x] >> 5) & 0x3fu;
        unsigned b =  row[x] >> 11;
#endif
        // We must use 6:6:6 output format.
        b = (b << 1) | (b >> 4);
        r = (r << 1) | (r >> 4);
        fputc(r, out); fputc(g, out); fputc(b, out);
        x++;
      }
      y++;
    }   
    fclose(out);
  }
}

void spidpi::sckEdge(svBitVecVal in) {
  // Data arrives MS bit first.
  inByte = (inByte << 1) | (in & 1u);
  if (++nbits >= 8u) {
    bool cmdComplete = true;  // Assume all unrecognised commands are a single byte long.
    bool emitImage = false;

    // Collect command bytes until we have the full command.
    cmd[cmdLen++] = inByte;
    if (pixWriting) {
      // We support only the 5:6:5 format presently, so two bytes per pixel.
      cmdComplete = (cmdLen >= 2);
      if (cmdComplete) {
        // Ensure that we do not write out-of-bounds.
        if (colAdr < width && rowAdr < height) {
          uint16_t pixCol = ((uint16_t)cmd[0] << 8) | cmd[1];
          frameBuf[rowAdr][colAdr] = pixCol;
        }
        // Advance to the next pixel.
        if (++colAdr > xEnd) {
          // Update the output image.
          emitImage = true;
          colAdr = xStart;
          if (++rowAdr > yEnd) {
            pixWriting = false;
          }
          logText("colAdr 0x%0x rowAdr 0x%0x", colAdr, rowAdr);
        }
      }
    } else {
      logText("%0x (cmdLen %0x cmd %0x)\n", inByte, cmdLen, cmd[0]);
      // If we now have a complete command then execute it.
      switch (cmd[0u]) {
        case ST7735_NOP:     cmdComplete = true; break;
        case ST7735_SWRESET: reset(); cmdComplete = true; break;
        case ST7735_SLPIN:   cmdComplete = true; break;
        case ST7735_SLPOUT:  cmdComplete = true; break;
        case ST7735_COLMOD:  cmdComplete = (cmdLen >= 2); break;
        case ST7735_FRMCTR1: cmdComplete = (cmdLen >= 4); break;
        case ST7735_FRMCTR2: cmdComplete = (cmdLen >= 4); break;
        case ST7735_FRMCTR3: cmdComplete = (cmdLen >= 7); break;
        case ST7735_MADCTL:  cmdComplete = (cmdLen >= 2); break;
        case ST7735_DISSET5: cmdComplete = (cmdLen >= 3); break;
        case ST7735_INVCTR:  cmdComplete = (cmdLen >= 2); break;
        case ST7735_PWCTR1:  cmdComplete = (cmdLen >= 3); break;
        case ST7735_PWCTR2:  cmdComplete = (cmdLen >= 2); break;
        case ST7735_PWCTR3:  cmdComplete = (cmdLen >= 3); break;
        case ST7735_PWCTR4:  cmdComplete = (cmdLen >= 3); break;
        case ST7735_PWCTR5:  cmdComplete = (cmdLen >= 3); break;
        case ST7735_PWCTR6:  cmdComplete = (cmdLen >= 3); break;
        case ST7735_VMCTR1:  cmdComplete = (cmdLen >= 3); break;

        // Display inversion.
        case ST7735_INVOFF: break;
        case ST7735_INVON:  break;

        // Gamma.
        // case ST7735_GAMSET:  cmdComplete = (cmdLen >= 2); break;
        case ST7735_GMCTRP1: cmdComplete = (cmdLen >= 17); break;
        case ST7735_GMCTRN1: cmdComplete = (cmdLen >= 17); break;

        case ST7735_CASET:
          cmdComplete = (cmdLen >= 5);
          if (cmdComplete) {
            xStart = (cmd[1] << 8) | cmd[2];
            xEnd   = (cmd[3] << 8) | cmd[4];
            logText("CASET 0x%0x,%0x\n", xStart, xEnd);
          }
          break;
        case ST7735_RASET:
          cmdComplete = (cmdLen >= 5);
          if (cmdComplete) {
            yStart = (cmd[1] << 8) | cmd[2];
            yEnd   = (cmd[3] << 8) | cmd[4];
            logText("RASET 0x%0x,%0x\n", yStart, yEnd);
          }
          break;

        case ST7735_NORON: break;

        // Display On/Off.
        case ST7735_DISPON:  break;
        case ST7735_DISPOFF: break;

        // RAM Writing.
        case ST7735_RAMWR:
          logText("RAMWR 0x%0x,%0x : 0x%0x,%0x\n", xStart, xEnd, yStart, yEnd);
          colAdr = xStart;
          rowAdr = yStart;
          pixWriting = true;
          cmdComplete = true;
          break;
        // RAM Reading.
        case ST7735_RAMRD:
          break;

        // Driver code does attempt to issue command 0x84 (source inspection),
        // because it provides another parameter to PWCTR1.
        default:
          break;
      }
    }

    // Emit an updated image?
    if (emitImage) {
      writePPM();
    }

    // Update command parser.
    if (cmdComplete) cmdLen = 0u;
    nbits = 0u;
  }
}

// Interface is using vanilla C for now.
extern "C" {
void *spidpi_create() {
  spidpi *ctx = new spidpi;
  return (void*)ctx;
}

void spidpi_sckEdge(void *ctx_v, svBitVecVal copi) {
  spidpi *ctx = (spidpi*)ctx_v;
  ctx->sckEdge(copi);
}
};



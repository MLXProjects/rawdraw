//Copyright (c) 2017 <>< Charles Lohr - Under the MIT/x11 or NewBSD License you choose.

#ifndef _CNFGFBDEVDRIVER_C
#define _CNFGFBDEVDRIVER_C

#include "CNFG.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>

#define CNFG_FBDEV_DEBUG 1

/* debug macro, works like this when enabled and set to at least 1:
 * prints the calling function's name without newline (to prevent flushing to stdout)
 * prints the message specified as argument (with extra args if any)
 * prints a newline to end it all (and to trigger stdout flush)
 * if debug level = 2, shows function's file name
 * if debug level >= 3, shows function's file name and line
 */
#ifdef CNFG_FBDEV_DEBUG
#if CNFG_FBDEV_DEBUG >= 3
#define CNFG_LOGD(...) { printf("%s:%d/%s ", __FILE__, __LINE__, __FUNCTION__); printf(__VA_ARGS__); printf("\n"); }
#elif CNFG_FBDEV_DEBUG == 2
#define CNFG_LOGD(...) { printf("%s/%s ", __FILE__, __FUNCTION__); printf(__VA_ARGS__); printf("\n"); }
#elif CNFG_FBDEV_DEBUG == 1
#define CNFG_LOGD(...) { printf("%s ", __FUNCTION__); printf(__VA_ARGS__); printf("\n"); }
#endif //CNFG_FBDEV_DEBUG level
#else
#define CNFG_LOGD(...) /* no debug */
#endif //CNFG_FBDEV_DEBUG

static int fb_fd;
static struct fb_fix_screeninfo fb_fix;
static struct fb_var_screeninfo fb_var;
static void *fb_buffer, *fb_curbuffer;
static int req_w, req_h;
static uint32_t fb_forecolor;
static short fb_doublebuf;
static short fb_pixsz;

static uint16_t pixel_color16(uint32_t RGB)
{
	unsigned int r = (RGB & 0x00FF0000) >> 16;
	unsigned int g = (RGB & 0x0000FF00) >> 8;
	unsigned int b =  RGB & 0x000000FF;
	return (r >> 3 << 11) + (g >> 2 << 5) + (b >> 3);
}

static uint32_t pixel_color32(uint32_t RGB)
{
	/* get color in fbdev-specified color order */
	uint8_t r = ( RGB >> fb_var.blue.offset ) & 0xFF;
	uint8_t g = ( RGB >> fb_var.green.offset ) & 0xFF;
	uint8_t b = ( RGB >> fb_var.red.offset ) & 0xFF;
	unsigned long color = (r<<16)|(g<<8)|(b);
	return color;
}

static void pixel_draw16(int x, int y, uint16_t color){
	uint16_t *fb_offset = (uint16_t*)(fb_curbuffer + (y * fb_fix.line_length) + (x*fb_pixsz));
	*fb_offset = color;
}

static void pixel_draw32(int x, int y, uint32_t color){
	uint32_t *fb_offset = (uint32_t*)(fb_curbuffer + (y * fb_fix.line_length) + (x*fb_pixsz));
	*fb_offset = color;
}

void CNFGGetDimensions( short * x, short * y )
{
	*x = fb_var.xres;
	*y = fb_var.yres;
}

static void InternalLinkScreenAndGo( const char * WindowName )
{
}

void CNFGSetupFullscreen( const char * WindowName, int screen_no )
{
	CNFGSetup( WindowName, 640, 480 );
}


void CNFGTearDown()
{
	if (fb_buffer != MAP_FAILED){
		CNFG_LOGD("munmap framebuffer memory");
		munmap(fb_buffer, fb_fix.smem_len);
	}
	CNFG_LOGD("closing framebuffer fd");
	close(fb_fd);
}

int CNFGSetup( const char * WindowName, int sw, int sh )
{
	fb_buffer = MAP_FAILED; /* to prevent munmap if init fails before mmapping */
	req_w = sw;
	req_h = sh;
	fb_fd = open("/dev/fb0", O_RDWR, 0);
	if (fb_fd < 1){
		CNFG_LOGD("failed to open framebuffer at /dev/fb0, retrying with secondary path");
		fb_fd = open("/dev/graphics/fb0", O_RDWR, 0);
	}
	if (fb_fd < 1){
		CNFG_LOGD("failed to open framebuffer");
		exit(1);
	}
	atexit( CNFGTearDown );
	CNFG_LOGD("requesting fixed & variable information");
	ioctl(fb_fd, FBIOGET_FSCREENINFO, &fb_fix);
	ioctl(fb_fd, FBIOGET_VSCREENINFO, &fb_var); 
	CNFG_LOGD("CNFG FRAMEBUFFER INFORMATIONS:");
	CNFG_LOGD("VAR");
	CNFG_LOGD(" xres                : %i", fb_var.xres);
	CNFG_LOGD(" yres                : %i", fb_var.yres);
	CNFG_LOGD(" xres_virtual        : %i", fb_var.xres_virtual);
	CNFG_LOGD(" yres_virtual        : %i", fb_var.yres_virtual);
	CNFG_LOGD(" xoffset             : %i", fb_var.xoffset);
	CNFG_LOGD(" yoffset             : %i", fb_var.yoffset);
	CNFG_LOGD(" bits_per_pixel      : %i", fb_var.bits_per_pixel);
	CNFG_LOGD(" grayscale           : %i", fb_var.grayscale);
	CNFG_LOGD(" red                 : %i, %i, %i",
		fb_var.red.offset, fb_var.red.length, fb_var.red.msb_right);
	CNFG_LOGD(" green               : %i, %i, %i",
		fb_var.green.offset, fb_var.green.length, fb_var.red.msb_right);
	CNFG_LOGD(" blue                : %i, %i, %i",
		fb_var.blue.offset, fb_var.blue.length, fb_var.red.msb_right);
	CNFG_LOGD(" transp              : %i, %i, %i",
		fb_var.transp.offset, fb_var.transp.length, fb_var.red.msb_right);
	CNFG_LOGD(" nonstd              : %i", fb_var.nonstd);
	CNFG_LOGD(" activate            : %i", fb_var.activate);
	CNFG_LOGD(" height              : %i", fb_var.height);
	CNFG_LOGD(" width               : %i", fb_var.width);
	CNFG_LOGD(" accel_flags         : %i", fb_var.accel_flags);
	CNFG_LOGD(" pixclock            : %i", fb_var.pixclock);
	CNFG_LOGD(" left_margin         : %i", fb_var.left_margin);
	CNFG_LOGD(" right_margin        : %i", fb_var.right_margin);
	CNFG_LOGD(" upper_margin        : %i", fb_var.upper_margin);
	CNFG_LOGD(" lower_margin        : %i", fb_var.lower_margin);
	CNFG_LOGD(" hsync_len           : %i", fb_var.hsync_len);
	CNFG_LOGD(" vsync_len           : %i", fb_var.vsync_len);
	CNFG_LOGD(" sync                : %i", fb_var.sync);
	CNFG_LOGD(" rotate              : %i", fb_var.rotate);
	CNFG_LOGD("FIX");
	CNFG_LOGD(" id                  : %s", fb_fix.id);
	CNFG_LOGD(" smem_len            : %i", fb_fix.smem_len);
	CNFG_LOGD(" type                : %i", fb_fix.type);
	CNFG_LOGD(" type_aux            : %i", fb_fix.type_aux);
	CNFG_LOGD(" visual              : %i", fb_fix.visual);
	CNFG_LOGD(" xpanstep            : %i", fb_fix.xpanstep);
	CNFG_LOGD(" ypanstep            : %i", fb_fix.ypanstep);
	CNFG_LOGD(" ywrapstep           : %i", fb_fix.ywrapstep);
	CNFG_LOGD(" line_length         : %i", fb_fix.line_length);
	CNFG_LOGD(" accel               : %i", fb_fix.accel);
	/* check for non-16/32 framebuffer */
	if ((fb_var.bits_per_pixel != 32) && (fb_var.bits_per_pixel != 16)) {
		/* non 32/16bit colorspace is not supported */
		CNFG_LOGD("bits_per_pixel=%d not supported", fb_var.bits_per_pixel);
		exit(1);
	}
	fb_pixsz = fb_var.bits_per_pixel>>3;
	CNFG_LOGD("mmap framebuffer memory");
	fb_buffer = (void*) mmap(0, fb_fix.smem_len, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, 0);
	if (fb_buffer == MAP_FAILED){
		CNFG_LOGD("failed to map framebuffer memory");
		exit(1);
	}
	fb_curbuffer = fb_buffer;

	fb_doublebuf=0;
	if (fb_var.yres_virtual<fb_var.yres*2){
		/* request double buffer */
		fb_var.yres_virtual=fb_var.yres*2;
		fb_var.activate = FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE;
		ioctl(fb_fd, FBIOPUT_VSCREENINFO, &fb_var);

		/* update vars */
		ioctl(fb_fd, FBIOGET_FSCREENINFO, &fb_fix);
		ioctl(fb_fd, FBIOGET_VSCREENINFO, &fb_var);
		if (fb_var.yres_virtual>=fb_var.yres*2){
			/* support double buffering */
			fb_doublebuf=1;
		}
	}
	else{
		/* already supports double buffer */
		fb_var.activate = FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE;
		ioctl(fb_fd, FBIOPUT_VSCREENINFO, &fb_var);
		fb_doublebuf=1;
	}
	CNFG_LOGD("Double Buffering = %s",fb_doublebuf?"yes":"no");

	/* update vars */
	ioctl(fb_fd, FBIOGET_FSCREENINFO, &fb_fix);
	ioctl(fb_fd, FBIOGET_VSCREENINFO, &fb_var);
	/* default foreground color is black */
	fb_forecolor=0;
	/* clear framebuffer, optional */
	CNFGClearFrame();
	CNFGSwapBuffers();
	usleep(3000*1000);
	CNFG_LOGD("framebuffer init done");
	return 0;
}

int CNFGHandleInput()
{
	/* TODO: implement /dev/input parsing */
	usleep(16);
	return 1;
}

#ifndef CNFGRASTERIZER
void CNFGBlitImage( uint32_t * data, int x, int y, int w, int h )
{
	CNFG_LOGD("blitting image at %d, %d (%dx%d)", x, y, w, h);
	/* basic safety checks */
	if (x>=fb_var.xres) return;
	if (x+w>=fb_var.xres) w = fb_var.xres;
	if (y>=fb_var.yres) return;
	if (y+h>=fb_var.yres) h = fb_var.yres;
	/* let's draw in the slowest way ever */
	int dx, dy;
	if (fb_pixsz==4){ /* 32 bpp */
		int offset_linew = fb_fix.line_length-(x*fb_pixsz);
		/* put image directly */
		for (dy=0;dy<h;dy++){
			for (dx=0;dx<w;dx++){
				/* for some reason, the commented code doesn't work properly
				 * despite being exactly the same as the function being called.
				 * I love C. */
				//uint32_t *fb_offset = (uint32_t*)fb_curbuffer + ((y+dy)*fb_fix.line_length) + ((x+dx)*fb_pixsz);
				//*fb_offset = *(data+(dy*w)+dx);
				pixel_draw32(x+dx, y+dy, *(data+(dy*w)+dx));
			}
		}
	}
	else {
		/* adapt 32bpp image to 16bpp buffer */
		for (dy=0;dy<h;dy++){
			for (dx=0;dx<w;dx++){
				pixel_draw16(x+dx, y+dy, pixel_color16(*(data+(dy*w)+dx)));
			}
		}
	}
}
#endif

void CNFGUpdateScreenWithBitmap( uint32_t * data, int w, int h )
{
	/* basic safety checks */
	/*
	if (w>=fb_var.xres) w = fb_var.xres;
	if (h>=fb_var.yres) h = fb_var.yres;
	int x, y;
	if (fb_var.bits_per_pixel==32){
		for (y=0;y<h;y++){
			for (x=0;x<w;x++)
			{
				((uint32_t*)fb_curbuffer)[x*y] = data[x*y];
			}
		}
	}
	else {
	}*/
	//CNFGSwapBuffers();
}

uint32_t CNFGColor( uint32_t RGB )
{
	fb_forecolor = pixel_color32(RGB);
	return fb_forecolor;
}

void CNFGClearFrame()
{
	int x,y;
	if (fb_var.bits_per_pixel==32){
		uint32_t bgcolor_32 = pixel_color32(CNFGBGColor);
		for (y=0; y<fb_var.yres; y++){
			for (x=0; x<fb_var.xres; x++){
				*(uint32_t*)(fb_curbuffer+(y * fb_fix.line_length)+(x*fb_pixsz))=bgcolor_32;
			}
		}
	}
	else {
		uint16_t bgcolor_16 = pixel_color16(CNFGBGColor);
		for (y=0; y<fb_var.yres; y++){
			for (x=0; x<fb_var.xres; x++){
				*(uint16_t*)(fb_curbuffer+(y * fb_fix.line_length)+(x*fb_pixsz))=bgcolor_16;
			}
		}
	}
}

void CNFGSwapBuffers()
{
	if (fb_doublebuf){
		fb_curbuffer = fb_buffer + (fb_var.yoffset * fb_fix.line_length);
		if (fb_var.yoffset==0){
			fb_var.yoffset = fb_var.yres;
		}
		else{
			fb_var.yoffset=0;
		}
	}
	fb_var.activate = FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE;
	if (ioctl(fb_fd, FBIOPAN_DISPLAY, &fb_var)!=0){
		ioctl(fb_fd, FBIOPUT_VSCREENINFO, &fb_var);
	}
}

void CNFGTackSegment( short x1, short y1, short x2, short y2 )
{
	int i,dx,dy,sdx,sdy,dxabs,dyabs,x,y,px,py;

	dx=x2-x1;			//Delta x
	dy=y2-y1;			//Delta y
	dxabs=abs(dx);		//Absolute delta
	dyabs=abs(dy);		//Absolute delta
	sdx=(dx>0)?1:-1;	//signum function
	sdy=(dy>0)?1:-1;	//signum function
	x=dyabs>>1;
	y=dxabs>>1;
	px=x1;
	py=y1;

	if (dxabs>=dyabs)
	{
		if (fb_pixsz==4){ /* 32 bpp */
			for(i=0;i<dxabs;i++)
			{
				y+=dyabs;
				if (y>=dxabs)
				{
					y-=dxabs;
					py+=sdy;
				}
				px+=sdx;
				pixel_draw32(px,py,fb_forecolor);
			}
		}
		else { /* 16 bpp */
			for(i=0;i<dxabs;i++)
			{
				y+=dyabs;
				if (y>=dxabs)
				{
					y-=dxabs;
					py+=sdy;
				}
				px+=sdx;
				pixel_draw16(px,py,fb_forecolor);
			}
		}
	}
	else
	{
		if (fb_pixsz==4){ /* 32 bpp */
			for(i=0;i<dyabs;i++)
			{
				x+=dxabs;
				if (x>=dyabs)
				{
					x-=dyabs;
					px+=sdx;
				}
				py+=sdy;
				pixel_draw32(px,py,fb_forecolor);
			}
		}
		else { /* 16 bpp */
		uint16_t forecolor_16 = pixel_color16(fb_forecolor);
			for(i=0;i<dyabs;i++)
			{
				x+=dxabs;
				if (x>=dyabs)
				{
					x-=dyabs;
					px+=sdx;
				}
				py+=sdy;
				pixel_draw16(px,py,forecolor_16);
			}
		}
	}
}

void CNFGTackPixel( short x1, short y1 )
{
	if (fb_pixsz==4) /* 32 bpp */
		pixel_draw32(x1, y1, fb_forecolor);
	else /* 16bpp */
		pixel_draw16(x1, y1, pixel_color16(fb_forecolor));
}

void CNFGTackRectangle( short x1, short y1, short x2, short y2 )
{
	/* basic safety checks */
	if (x1>=fb_var.xres) return;
	if (x2>=fb_var.xres) x2 = fb_var.xres;
	if (y1>=fb_var.yres) return;
	if (y2>=fb_var.yres) y2 = fb_var.yres;
	/* let's draw in the slowest way ever */
	int x, y;
	if (fb_pixsz==4){ /* 32bpp */
		for (y=y1;y<y2;y++){
			for (x=x1;x<x2;x++)
			{
				pixel_draw32(x, y, fb_forecolor);
			}
		}
	}
	else {
		uint16_t forecolor_16 = pixel_color16(fb_forecolor);
		for (y=y1;y<y2;y++){
			for (x=x1;x<x2;x++)
			{
				pixel_draw16(x, y, forecolor_16);
			}
		}
	}
}

void CNFGTackPoly( RDPoint * points, int verts )
{
}

#endif //_CNFGFBDEVDRIVER_C

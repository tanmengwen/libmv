#include <stdio.h>
#include <stdlib.h>
char* OPTIONS = "h";

void print_help_and_exit(char* progname) {
	printf("print an exr file as a numpy literal\n");
	printf("usage: %s <file.exr>\n", progname);
	printf("  -h           this message\n");
	exit(-1);
}

void exit_error(char* progname, char *msg) {
	fprintf(stderr, msg);
	print_help_and_exit(progname);
}
#define ERROR(msg) exit_error(argv[0], "ERROR: " msg "\n")

#include <ImfArray.h>
#include <ImfRgbaFile.h>
#include <ImfInputFile.h>
#include <ImathBox.h>

using namespace Imf;
using namespace Imath;

void
readRGBA (const char fileName[],
		Array2D<float> &rPixels,
		Array2D<float> &gPixels,
		Array2D<float> &bPixels,
		Array2D<float> &aPixels,
		int &width, int &height)
{
	InputFile file (fileName);
	Box2i dw = file.header().dataWindow();
	width = dw.max.x - dw.min.x + 1;
	height = dw.max.y - dw.min.y + 1;
	rPixels.resizeErase (height, width);
	gPixels.resizeErase (height, width);
	bPixels.resizeErase (height, width);
	aPixels.resizeErase (height, width);
	FrameBuffer frameBuffer;
	frameBuffer.insert ("R", // name
			Slice (FLOAT, // type
				(char *) (&rPixels[0][0] - // base
					dw.min.x -
					dw.min.y * width),
				sizeof (rPixels[0][0]) * 1, // xStride
				sizeof (rPixels[0][0]) * width,// yStride
				1, 1, // x/y sampling
				3.0)); // fillValue
	frameBuffer.insert ("G", // name
			Slice (FLOAT, // type
				(char *) (&gPixels[0][0] - // base
					dw.min.x -
					dw.min.y * width),
				sizeof (gPixels[0][0]) * 1, // xStride
				sizeof (gPixels[0][0]) * width,// yStride
				1, 1, // x/y sampling
				4.0)); // fillValue
	frameBuffer.insert ("B", // name
			Slice (FLOAT, // type
				(char *) (&bPixels[0][0] - // base
					dw.min.x -
					dw.min.y * width),
				sizeof (bPixels[0][0]) * 1, // xStride
				sizeof (bPixels[0][0]) * width,// yStride
				1, 1, // x/y sampling
				5.0)); // fillValue
	frameBuffer.insert ("A", // name
			Slice (FLOAT, // type
				(char *) (&aPixels[0][0] - // base
					dw.min.x -
					dw.min.y * width),
				sizeof (aPixels[0][0]) * 1, // xStride
				sizeof (aPixels[0][0]) * width,// yStride
				1, 1, // x/y sampling
				6.0)); // fillValue
	file.setFrameBuffer (frameBuffer);
	file.readPixels (dw.min.y, dw.max.y);
}

int main(int argc, char* argv[])
{
	char *exr_fn=NULL;

	char argchar;
	while ((argchar = getopt (argc, argv, OPTIONS)) != -1) {
		switch (argchar) {
		case 'h': print_help_and_exit(argv[0]);   break;
		default:  print_help_and_exit(argv[0]);
		}
	}

	if (argc != 2)
		ERROR("need an exr file to print");

	exr_fn = argv[1];

	int width, height;
	Array2D<float> aPixels;
	Array2D<float> bPixels;
	Array2D<float> gPixels;
	Array2D<float> rPixels;
	readRGBA(exr_fn, rPixels, gPixels, bPixels, aPixels, width, height);
	fprintf(stderr, "Read EXR file; (%d,%d)\n", width,height);

	printf("imgA=[\n");
	for (int j=0; j<height; j++) {
		printf("[");
		for (int i=0; i<width; i++) {
			printf("%f,", aPixels[j][i]);
		}
		printf("],\n");
	}
	printf("]\n");
	printf("imgB=[\n");
	for (int j=0; j<height; j++) {
		printf("[");
		for (int i=0; i<width; i++) {
			printf("%f,", bPixels[j][i]);
		}
		printf("],\n");
	}
	printf("]\n");
	printf("imgG=[\n");
	for (int j=0; j<height; j++) {
		printf("[");
		for (int i=0; i<width; i++) {
			printf("%f,", gPixels[j][i]);
		}
		printf("],\n");
	}
	printf("]\n");
	printf("imgR=[\n");
	for (int j=0; j<height; j++) {
		printf("[");
		for (int i=0; i<width; i++) {
			printf("%f,", rPixels[j][i]);
		}
		printf("],\n");
	}
	printf("]\n");
}

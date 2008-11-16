    **************************************************************
                                HOMEST
                              version 1.0
                          By Manolis Lourakis

                     Institute of Computer Science
            Foundation for Research and Technology - Hellas
                       Heraklion, Crete, Greece
    **************************************************************


GENERAL
This is homest, a copylefted C implementation of a technique for non-linear, robust
homography estimation from matched image point features. homest requires my levmar
Levenberg-Marquardt non-linear least squares implementation and LAPACK, available
from http://www.ics.forth.gr/~lourakis/levmar and http://www.netlib.org/clapack,
respectively. Also, it implements robust regression techniques for coping with
outliers. Note that homest does *not* include any means for detecting and matching
point features between images. Such functionality can be supplied by other software
such as D. Lowe's SIFT or (in some cases) S. Birchfield's KLT.

Briefly, the technique implemented by homest is the following:

1) Normalization of point coordinates to improve conditioning as
   described in
   R.I. Hartley "In Defense of the Eight-Point Algorithm",
   IEEE Trans. on PAMI, Vol. 19, No. 6, pp. 580-593, June 1997.

2) Least Median of Squares (LMedS) linear fitting to detect outliers,
   see P.J. Rousseeuw, "Least Median of Squares Regression", Journal
   of the American Statistics Association, Vol. 79, No. 388, pp. 871-880,
   Dec. 1984. To ensure adequate spatial distribution of point quadruples
   over the image, LMedS random sampling employs the bucketing technique
   proposed in
   Z. Zhang, R. Deriche, O. Faugeras, Q.T. Luong, 
   "A Robust Technique for Matching Two Uncalibrated Images Through the
   Recovery of the Unknown Epipolar Geometry", INRIA RR-2273, May 1994 

3) Non-linear refinement of the linear homography estimate by minimizing either:
   * the symmetric homographic transfer error between the images
   * the Sampson error (see P.D. Sampson, "Fitting Conic Sections to ``very
     scattered'' Data: An Iterative Refinement of the Bookstein Algorithm",
     CGIP, Vol. 18, Is. 1, pp. 97-108, Jan. 1982.)
   The minimization is performed using the Levenberg-Marquardt algorithm, see
   M.I.A. Lourakis, "levmar: Levenberg-Marquardt Nonlinear Least Squares
   Algorithms in C/C++", http://www.ics.forth.gr/~lourakis/levmar/

For more details on homographies, projective geometry and such,
refer to R. Hartley and A. Zisserman, "Multiple View Geometry in
Computer Vision", Cambridge University Press, 2000-3


COMPILATION
 - Make sure that levmar and LAPACK are installed on your system. Installation
   details for these can be found at their respective URLs mentioned above.

 - Edit the appropriate makefile to specify the location of your compiled
   levmar and LAPACK libraries.

 - On a Linux/Unix system, typing "make" will build both homest and the demo
   program using gcc.

 - Under Windows and if Visual C is installed & configured for command line
   use, type "nmake /f Makefile.vc" in a cmd window to build homest and the
   demo program. In case of trouble, read the comments on top of Makefile.vc

USE
homest() is the main routine for estimating a homography. See the comments in
homest.c for an explanation of its arguments; homest_demo.c illustrates an
example of using homest() with matched point pairs that are read from a text
file. An example of such point pairs is included in subdirectory test.
Typing 
./homest_demo  test/matches.txt 

should produce output similar to

./homest_demo: estimated homography [15 outliers out of 1046 matches]

0.4743672 0.1694731 -21.47851 
-0.09850428 0.5042611 82.7945 
0.0001071338 -1.036262e-05 0.5391076 

Homography RMS and RMedS errors for input points: 31.5007 0.683278

Send your comments/bug reports to lourakis@ics.forth.gr

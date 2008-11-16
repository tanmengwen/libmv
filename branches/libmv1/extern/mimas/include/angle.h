//
// angle routines and classes
// stuart meikle Wed Mar  8 15:45:13 2000
//

// beats messing around with angle calcs all the time!

#ifndef ANGLE_H
#define ANGLE_H

#include <cmath>

namespace mimas {
	
  /** Angle class.
      Providing different operators for angles. The operators always will keep
      the value within \f$[-pi,pi)\f$.
      @author Stuart Meikle (stu@stumeikle.org)
      @date Wed Mar  8 15:45:13 2000 BST */
  class	angle
    {
    private:
      /** The actual value.
	  \f$-pi\le\mathrm{angle}<pi\f$ must hold. */
      double _angle;
	
      void arcTan2(double y, double x );
      void normalize(void);
    public:

      /// Default constructor.
      angle(void): _angle(0) {}
      /** Constructor.
	  @param aAngle Initial value. \f$\mathrm{value}\in[-pi,pi)\f$ must
	  hold. */
      angle( double aAngle ): _angle(aAngle) {}

      angle( double y, double x ) { arcTan2(y,x); }
	
      //this next is almost not an angle function , but it needs to live
      //within the namespace... DOESNT operate on 'this'

      //overload casting an angle to a double:
      operator double() const {return _angle;}
	
      //basic ops overloading:
      angle operator+( angle &a2 );
      angle operator-( angle &a2 );
      angle operator-(void) { return angle( -_angle ); }
      /* angle        operator+=(const angle& right);
	 angle        operator+=(const double& right); */
      angle&        operator+=(const angle& right);
      angle&        operator+=(const double& right);
    };
    
};

#endif

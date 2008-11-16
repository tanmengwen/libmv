#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <exception>
#include <sstream>
#include <string>

namespace mimas {

/** Exception class deriving from std::exception.
    This class provides a syntax similar to output-streams for convenience.
    For compability with other libraries it is inheriting the class
    \c std::exception.
    
    Here is an example how to use an instance of this class (by invoking the
    macro MMERROR):
    \code
    void test( int i ) throw (exception)
    {
      MMERROR( i > 0, exception, ,
               "Parameter for method test must be greater than zero (but was "
               << i << ")." );
    }
    
    int main(void)
    {
      try {
        test( -5 );
      } catch ( exception &e ) {
        cerr << e.what() << endl;
      };
      return 0;
    }
    \endcode
    Mind that the macro uses a variable with the name \c _e. Make sure, that
    you don't use this variable-name in any of the macro-arguments!

    @author Jan Wedekind (jan@wedesoft.de)
    @date Mon Aug 23 14:37:05 UTC 2004 */
class mimasexception: public std::exception
{
 public:
  /// Constructor.
  mimasexception(void) {}
  /// Copy constructor.
  mimasexception( mimasexception &e ): std::exception( e )
  { message << e.message.str(); }
  /// Destructor.
  virtual ~mimasexception(void) throw() {}
  ///
  template< typename T >
  std::ostream &operator<<( const T &t ) { message << t; return message; }
  /** Interface for manipulators.
      Manipulators such as \c std::endl and \c std::hex use these
      functions in constructs like "mimasexception e; e << std::endl".
      For more information, see the iomanip header. */
  std::ostream &operator<<( std::ostream& (*__pf)( std::ostream&) )
    { (*__pf)( message ); return message; }
  /// Returns error message (not thread safe).
  virtual const char* what(void) const throw() {
    temp = message.str();
    return temp.c_str();
  }
 protected:
  /// Memory-stream containing the error message.
  std::ostringstream message;
  /** Temporary to do null-termination.
      The method \c what() requires a null-terminated string. */
  static std::string temp;
};

};

#define MMERROR( condition, class, params, message )   \
  if ( !( condition ) ) {                              \
    class _e params;                                    \
    _e << message;                                      \
    throw _e;                                           \
  };

#endif

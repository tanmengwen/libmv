#!/bin/sh
sed 's/boost::numeric::ublas::/u:/g' | \
sed 's/u:matrix<double, u:basic_row_major<unsigned int, int>, u:unbounded_array<double, std::allocator<double> > >/mat/g' | \
sed 's/\/usr\/local\/include\/boost-1_34\/boost\/numeric\/ublas/BOOST/g'

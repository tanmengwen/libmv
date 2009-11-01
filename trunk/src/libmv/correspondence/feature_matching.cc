// Copyright (c) 2009 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.


#include "libmv/correspondence/feature_matching.h"

// Compute candidate matches between 2 sets of features.  Two features a and b
// are a candidate match if a is the nearest neighbor of b and b is the nearest
// neighbor of a.
void FindCandidateMatches(const FeatureSet &left,
                          const FeatureSet &right,
                          Matches *matches) {
  int max_track_number = 0;
  for (size_t i = 0; i < left.features.size(); ++i) {
    int j, k;
    float distance; 
    right.tree.ApproximateNearestNeighborBestBinFirst(
      left.features[i].descriptor.coords.data(), 300, &j, &distance);
    left.tree.ApproximateNearestNeighborBestBinFirst(
      right.features[j].descriptor.coords.data(), 300, &k, &distance);
    
    // Left image is image 0, right is 1 for now.
    if (i == k) {
      // Both kdtrees matched the same feature, so it is probably a match.
      matches->Insert(0, max_track_number, &left.features[i]);
      matches->Insert(1, max_track_number, &right.features[j]);
      max_track_number++;
    }
  }
}

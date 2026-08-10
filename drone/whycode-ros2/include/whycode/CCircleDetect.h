#ifndef WHYCODE__CCIRCLEDETECT_H
#define WHYCODE__CCIRCLEDETECT_H

#include <math.h>
#include <memory>
#include "whycode/CRawImage.h"
#include "whycode/CTransformation.h"
#include "whycode/CNecklace.h"
#include "whycode/types.h"

#define INNER 0
#define OUTER 1

namespace whycode {

struct SSegSmall {
    float x, y;
    float m0, m1;
    float v0, v1;
};

class CCircleDetect {
public:
  //constructor, wi and he correspond to the image dimensions 
  CCircleDetect(bool id, int bits, int samples, bool draw, CTransformation *trans, CNecklace *decoder);

  // dynamic reconfigure of parameters
  void reconfigure(float ict,float fct,float art,float cdtr,float cdta, bool id, int minS, int maxS, int minS_outer, int minS_inner);

  //main detection method, implements Algorithm 2 of [1] 
  SMarker findSegment(CRawImage &image, SSegment init);

  void setDraw(bool draw);

  int debug;                  // debug level
  bool draw_, lastTrackOK;     // flags to draw results - used for debugging
  bool localSearch;           // used when selecting the circle by mouse click
  bool identify;              // attempt to identify segments

private:
  //local pattern search - implements Algorithm 1 of [1]
  bool examineSegment(CRawImage &image, SSegment *segmen, int ii, float areaRatio, bool is_inner = false);

  //calculate the pattern dimensions by means of eigenvalue decomposition, see 3.3 of [1]
  SSegment calcSegment(SSegment segment,int size,long int x,long int y,long int cm0,long int cm1,long int cm2);

  //cleanup the shared buffers - see 3.6 of [1] 
  void bufferCleanup(SSegment init);

  //change threshold if circle not detected, see 3.2 of [1]
  bool changeThreshold();

  //normalise the angle
  float normalizeAngle(float a);

  // adjust the dimensions of the image, when the image size changes
  void adjustDimensions(int wi, int he);

  bool ambiguityAndObtainCode(CRawImage &image);
  void ambiguityPlain();

  CTransformation *trans_;
  CNecklace *decoder_;
  
  int idBits;
  int idSamples;
  int hammingDist;
  bool track;
  int maxFailed;
  int numFailed;
  int threshold; 

  int minSize;
  int maxSize;
  int minSizeOuter;
  int minSizeInner;
  int lastThreshold;
  int thresholdBias; 
  int maxThreshold; 

  int thresholdStep;
  float circularTolerance;
  float circularityTolerance;
  float ratioTolerance;
  float centerDistanceToleranceRatio;
  int centerDistanceToleranceAbs;
  bool enableCorrections;

  int ID;
  int step;
  SSegment inner;
  SSegment outer;
  int lastOuterSize;            // outer segment size before summing
  int lastInnerSize;            // inner segment size
  float outerAreaRatio, innerAreaRatio, areasRatio;
  int queueStart, queueEnd, queueOldStart, numSegments;
  int expand[4];
  unsigned char *ptr;
  float diameterRatio;
  bool ownBuffer;

  static int width;
  static int height;
  static int len;
  static std::unique_ptr<int> buffer;
  static std::unique_ptr<int> queue;

  SEllipseCenters ellipse_centers;
  STrackedObject tracked_object;
};

}  // namespace whycode

#endif  // WHYCODE__CCIRCLEDETECT_H

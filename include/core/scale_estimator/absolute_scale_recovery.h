#ifndef _ABSOLUTE_SCALE_RECOVERY_H_
#define _ABSOLUTE_SCALE_RECOVERY_H_

#include <iostream>
#include <string>
#include <exception>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>

#include <opencv2/core.hpp>

#include "core/type_defines.h"
#include "core/defines.h"

#include "core/camera.h"
#include "core/mapping.h"
#include "core/landmark.h"
#include "core/frame.h"
#include "core/keyframes.h"

class AbsoluteScaleRecovery
{
private:
    std::shared_ptr<Camera> cam_;

public:
    AbsoluteScaleRecovery(const std::shared_ptr<Camera>& cam);
    ~AbsoluteScaleRecovery();

public:
    void runASR();

};

#endif
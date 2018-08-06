// Copyright (c) 2017 GeometryFactory Sarl (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
// You can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL$
// $Id$
// SPDX-License-Identifier: GPL-3.0+
//
// Author(s)     : Simon Giraudot

#ifndef CGAL_CLASSIFICATION_H
#define CGAL_CLASSIFICATION_H

#include <license/Classification.h>

#include <Classification/classify.h>
#include <Classification/Sum_of_weighted_features_classifier.h>
#include <Classification/ETHZ_random_forest_classifier.h>

#ifdef CGAL_LINKED_WITH_OPENCV
#include <Classification/OpenCV_random_forest_classifier.h>
#endif

#include <Classification/Color.h>
#include <Classification/Evaluation.h>
#include <Classification/Feature_base.h>
#include <Classification/Feature_set.h>
#include <Classification/Label.h>
#include <Classification/Label_set.h>
#include <Classification/Local_eigen_analysis.h>
#include <Classification/Planimetric_grid.h>
#include <Classification/Point_set_feature_generator.h>
#include <Classification/Point_set_neighborhood.h>

#include <Classification/Feature/Distance_to_plane.h>
#include <Classification/Feature/Echo_scatter.h>
#include <Classification/Feature/Eigen.h>
#include <Classification/Feature/Elevation.h>
#include <Classification/Feature/Hsv.h>
#include <Classification/Feature/Simple_feature.h>
#include <Classification/Feature/Vertical_dispersion.h>
#include <Classification/Feature/Verticality.h>

#endif // CGAL_CLASSIFICATION_H

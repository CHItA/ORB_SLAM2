#ifndef orb_g2o_CONFIG_H
#define orb_g2o_CONFIG_H

/* #undef orb_g2o_OPENMP */
/* #undef orb_g2o_SHARED_LIBS */

// give a warning if Eigen defaults to row-major matrices.
// We internally assume column-major matrices throughout the code.
#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
#  error "orb_g2o requires column major Eigen matrices (see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=422)"
#endif

#endif

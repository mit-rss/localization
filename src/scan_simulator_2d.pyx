import cython
import numpy as np
cimport numpy as np
from libcpp.vector cimport vector

cdef extern from "racecar_simulator/pose_2d.hpp" namespace "racecar_simulator":

    cdef struct Pose2D:
        double x
        double y
        double theta

cdef extern from "racecar_simulator/scan_simulator_2d.hpp"  namespace "racecar_simulator":

    cppclass ScanSimulator2D:
        ScanSimulator2D(
                int num_beams_,
                double field_of_view_,
                double scan_std_dev_,
                double ray_tracing_epsilon_,
                int theta_discretization_)

        void set_map(
                vector[double] & map_,
                size_t height,
                size_t width,
                double resolution,
                Pose2D & origin,
                double free_threshold)

        void scan(Pose2D & pose, double * scan_data)

cdef class PyScanSimulator2D:
    cdef ScanSimulator2D * thisptr;
    cdef int num_beams

    def __init__(
            self, 
            int num_beams_,
            double field_of_view_,
            double scan_std_dev_,
            double ray_tracing_epsilon_,
            int theta_discretization_):
        self.num_beams = num_beams_
        self.thisptr = new ScanSimulator2D(
                num_beams_,
                field_of_view_,
                scan_std_dev_,
                ray_tracing_epsilon_,
                theta_discretization_)

    def __dealloc__(self):
        del self.thisptr

    def set_map(
            self,
            np.ndarray[double, ndim=1, mode='c'] map_np, 
            size_t height, 
            size_t width, 
            double resolution, 
            (double, double, double) origin_tuple,
            double free_threshold):

        cdef origin = Pose2D(origin_tuple[0], origin_tuple[1], origin_tuple[2])

        cdef vector[double] map_
        map_.assign(&map_np[0], &map_np[0] + map_np.shape[0])

        self.thisptr.set_map(
                map_,
                height,
                width,
                resolution,
                origin,
                free_threshold)

    def scan(self, np.ndarray[double, ndim=2, mode="c"] poses):
        # Allocate the output vector
        cdef np.ndarray[double, ndim=2, mode="c"] scans = \
                np.empty([poses.shape[0], self.num_beams], np.double)

        # Allocate the for loops
        cdef Pose2D p

        for i in xrange(poses.shape[0]):
            p = Pose2D(poses[i,0], poses[i,1], poses[i,2])
            self.thisptr.scan(p, &scans[i,0])

        return scans

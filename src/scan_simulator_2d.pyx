import cython
import numpy as np
cimport numpy as np
from libcpp.vector cimport vector

cdef extern from "pose_2d.hpp" namespace "racecar_simulator":

    cdef struct Pose2D:
        double x
        double y
        double theta

cdef extern from "scan_simulator_2d.hpp"  namespace "racecar_simulator":

    cppclass ScanSimulator2D:
        ScanSimulator2D(
                int num_beams_,
                double field_of_view_,
                double scan_std_dev_,
                double ray_tracing_epsilon)

        void set_map(
                vector[double] & map_,
                size_t height,
                size_t width,
                double resolution,
                Pose2D & origin,
                double free_threshold)

        vector[double] scan(Pose2D & pose)

cdef class PyScanSimulator2D:
    cdef ScanSimulator2D * thisptr;

    def __init__(self, int num_beams, double field_of_view, double scan_std_dev, double ray_tracing_epsilon):
        self.thisptr = new ScanSimulator2D(num_beams, field_of_view, scan_std_dev, ray_tracing_epsilon)

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

    def scan(self, np.ndarray[double, ndim=2] poses):
        # Allocate the output vector
        cdef np.dnarray[double, ndim=2] scans = np.empty([poses.shape[0], num_beams], np.double)

        for i in range(poses.shape[0]):
            cdef p = Pose2D(poses[0], poses[1], poses[2])
            cdef vector[double] s = self.thisptr.scan(p)

            for j in range(s.size()):
                scans[i,j] = s[j]

        return scans

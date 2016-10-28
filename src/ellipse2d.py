#!/usr/bin/env python

import numpy

from math import sin,cos,atan2,pi,sqrt,isnan
import matplotlib.pyplot as plt


class Ellipse2d:
    def __init__(self, A=[None]*6):
        self.A = A
        if None in self.A:
            self.a = None
            self.b = None
            self.theta = None
            self.center = None
        else:
            self._calculate_parameters()

    def __str__(self):
        if self.A is not None:
          return "Ellipse: a={0}, b={1}, theta={2}, center={3}".format(self.a, self.b, self.theta, self.center)
        else:
          return "Ungenerated ellipse."

    def is_valid(self):
        if self.a == None or self.b == None or self.theta == None or self.center == None \
           or isnan(self.a) or isnan(self.b) or isnan(self.center[0]) or isnan(self.center[1]):            
          return False
        else:
          return True

    def _calculate_parameters(self):
        # Calculate the angle
        self.theta = pi / 2.0 + 0.5 * atan2(self.A[1], self.A[0] - self.A[2])

        # Calculate the center
        det = self.A[0] * self.A[2] - 0.25 * self.A[1] * self.A[1]
        self.center = ((-0.5 * self.A[2] * self.A[3] + 0.25 * self.A[1] * self.A[4]) / det,
                       (0.25 * self.A[1] * self.A[3] - 0.5 * self.A[0] * self.A[4]) / det)

        # Calculate the major and minor axes  
        #### this sometimes errors out due to the a mathdomain error when getting complex numbers at end of calculate interestion
        ####  Not sure where to isolate
        self.a = self._calculate_intersection(self.theta)
        self.b = self._calculate_intersection(self.theta + pi/2)

        # Do we need to swap them so that a > b?
        if self.a < self.b:
            self.a,self.b = self.b,self.a
            self.theta += pi/2

            # Make sure 0 <= theta < pi.  After the calculations
            # above, we know that pi/4 < theta < 5pi/4, so we have to
            # subtract at most pi to get it back into range.
            if self.theta > pi:
                self.theta -= pi

    def _calculate_intersection(self, theta):
        cos_theta = cos(theta)
        sin_theta = sin(theta)

        a = self.A[0] * cos_theta * cos_theta + \
            self.A[1] * sin_theta * cos_theta + \
            self.A[2] * sin_theta * sin_theta
        b = 2 * self.A[0] * self.center[0] * cos_theta + \
            self.A[1] * self.center[0] * sin_theta + \
            self.A[1] * self.center[1] * cos_theta + \
            2 * self.A[2] * self.center[1] * sin_theta + \
            self.A[3] * cos_theta + \
            self.A[4] * sin_theta
        c = self.A[0] * self.center[0] * self.center[0] + \
            self.A[1] * self.center[0] * self.center[1] + \
            self.A[2] * self.center[1] * self.center[1] + \
            self.A[3] * self.center[0] + \
            self.A[4] * self.center[1] + \
            self.A[5]
    
        # print "a:{0}, b:{1}, c:{2}".format(a, b, c)
        return abs((-b + sqrt(b*b - 4 * a * c)) / (2.0 * a))
    
    def fit(self, points):
        np = numpy.array(points)
        centroid = (numpy.mean(np[:, 0]), numpy.mean(np[:, 1]))
        centered_points = [(p[0] - centroid[0], p[1] - centroid[1])
                           for p in points]

        D1 = numpy.array([(p[0] * p[0],  # x^2
                           p[0] * p[1],  # xy
                           p[1] * p[1])  # y^2
                          for p in centered_points])
        D2 = numpy.array([(p[0],         # x
                           p[1],         # y
                           1.0)          # constant
                          for p in centered_points])
        S1 = numpy.dot(D1.transpose(), D1)
        S2 = numpy.dot(D1.transpose(), D2)
        S3 = numpy.dot(D2.transpose(), D2)
        T = -numpy.dot(numpy.linalg.inv(S3), S2.transpose())

        M = S1 + numpy.dot(S2, T)
        M = numpy.array([M[2, :] / 2, -M[1, :], M[0, :] / 2])
        eval,evec = numpy.linalg.eig(M)

        cond = 4 * evec[0, :] * evec[2, :] - evec[1, :] * evec[1, :]

        # The argmax might fail for Inf eigenvalues, or if all are negative
        self.A = numpy.concatenate((evec[:, numpy.argmax(cond)],
                                    numpy.dot(T, evec[:, numpy.argmax(cond)])))

        self.A[5] += self.A[0] * centroid[0] * centroid[0] + \
                     self.A[2] * centroid[1] * centroid[1] + \
                     self.A[1] * centroid[0] * centroid[1] - \
                     self.A[3] * centroid[0] - \
                     self.A[4] * centroid[1]
        self.A[4] -= 2 * self.A[2] * centroid[1] + \
                     self.A[1] * centroid[0]
        self.A[3] -= 2 * self.A[0] * centroid[0] + \
                     self.A[1] * centroid[1]

        try:
          self._calculate_parameters()
        except ValueError:
          # sometimes the ellipses are imaginary and errors out, ignore these
          pass

    def plot(self):
        # Plot the axes
        plt.scatter(self.center[0], self.center[1], color='red')

        if self.a < self.b:
            a_color = 'green'
            b_color = 'red'
        else:
            a_color = 'red'
            b_color = 'green'
    
        plt.plot([self.center[0] - self.a * cos(self.theta), self.center[0] + self.a * cos(self.theta)],
                 [self.center[1] - self.a * sin(self.theta), self.center[1] + self.a * sin(self.theta)],
                 color=a_color)
        plt.plot([self.center[0] - self.b * cos(self.theta + pi/2), self.center[0] + self.b * cos(self.theta + pi/2)],
                 [self.center[1] - self.b * sin(self.theta + pi/2), self.center[1] + self.b * sin(self.theta + pi/2)],
                 color=b_color)

        # Plot the ellipse
        e = [(self.a * cos(angle), self.b * sin(angle)) for angle in
             [float(x) * pi / 100.0 for x in xrange(200)]]
        plt.plot([self.center[0] + p[0] * cos(self.theta) - p[1] * sin(self.theta) for p in e],
                 [self.center[1] + p[0] * sin(self.theta) + p[1] * cos(self.theta) for p in e],
                 color='red')
        

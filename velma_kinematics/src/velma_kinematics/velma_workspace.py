## Inverse kinematics for KUKA LWR 4+ robot and WUT Velma robot
# @file velma_ik_geom.py
# @ingroup python_api

# Copyright (c) 2021, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import rospkg
import math
import numpy as np
import random
from velma_kinematics.velma_ik_geom import KinematicsSolverLWR4, KinematicsSolverVelma

class LWRWorkspace:
    def __init__(self):
        self.__cell_size = None
        self.__x_min = None
        self.__x_max = None
        self.__y_min = None
        self.__y_max = None
        self.__z_min = None
        self.__z_max = None
        self.__x_cells = None
        self.__y_cells = None
        self.__z_cells = None
        self.__ws_matrix = None

    def __getCellIdx(self, v, v_min, cells):
        idx = int(math.floor((v-v_min)/self.__cell_size))
        if idx < 0 or idx >= cells:
            return None
        # else:
        return idx

    def getCellIdxX(self, v):
        return self.__getCellIdx(v, self.__x_min, self.__x_cells)

    def getCellIdxY(self, v):
        return self.__getCellIdx(v, self.__y_min, self.__y_cells)

    def getCellIdxZ(self, v):
        return self.__getCellIdx(v, self.__z_min, self.__z_cells)

    def getCellIdx(self, pt):
        ix = self.getCellIdxX(pt.x())
        iy = self.getCellIdxY(pt.y())
        iz = self.getCellIdxZ(pt.z())
        if ix is None or iy is None or iz is None:
            return None
        # else:
        return (ix, iy, iz)

    def __getCellCenter(self, idx, v_min, cells):
        if idx is None or idx < 0 or idx >= cells:
            return None
        return v_min + (idx+0.5)*self.__cell_size

    def getCellCenterX(self, idx):
        return self.__getCellCenter(idx, self.__x_min, self.__x_cells)

    def getCellCenterY(self, idx):
        return self.__getCellCenter(idx, self.__y_min, self.__y_cells)

    def getCellCenterZ(self, idx):
        return self.__getCellCenter(idx, self.__z_min, self.__z_cells)

    @staticmethod
    def generate():
        result = LWRWorkspace()

        # Parameters
        samples_count = 2000000
        lim_dist = 0.2
        result.__cell_size = 0.08

        solv = KinematicsSolverLWR4()

        q_lim = [ [-2.96, 2.96],
            [-2.09, -0.2, 0.2, 2.09],
            [-2.96, 2.96],
            [-2.095, -0.2, 0.2, 2.095],
            [-2.96, 2.96],
            [-2.09, -0.2, 0.2, 2.09],
            [-2.96, 2.96] ]

        pt_list = []
        x_min = 1000
        x_max = -1000
        y_min = 1000
        y_max = -1000
        z_min = 1000
        z_max = -1000
        for idx in range(samples_count):
            q = []
            for q_idx in range(7):
                q.append( random.uniform(q_lim[q_idx][0]+lim_dist, q_lim[q_idx][-1]-lim_dist) )
            T_AB_E = solv.calculateFk( q )
            pt_list.append(T_AB_E.p)
            x_min = min(x_min, T_AB_E.p.x())
            x_max = max(x_max, T_AB_E.p.x())
            y_min = min(y_min, T_AB_E.p.y())
            y_max = max(y_max, T_AB_E.p.y())
            z_min = min(z_min, T_AB_E.p.z())
            z_max = max(z_max, T_AB_E.p.z())
            if idx%1000 == 0:
                print('generated samples: {} / {}'.format(idx, samples_count))

        result.__x_min = x_min
        result.__x_max = x_max
        result.__y_min = y_min
        result.__y_max = y_max
        result.__z_min = z_min
        result.__z_max = z_max

        result.__x_cells = int(math.ceil((result.__x_max - result.__x_min) / result.__cell_size))
        result.__y_cells = int(math.ceil((result.__y_max - result.__y_min) / result.__cell_size))
        result.__z_cells = int(math.ceil((result.__z_max - result.__z_min) / result.__cell_size))

        print('ranges: ({}, {}), ({}, {}), ({}, {})'.format(result.__x_min, result.__x_max,
                            result.__y_min, result.__y_max, result.__z_min, result.__z_max))
        print('cells: {}, {}, {}, total: {}'.format(result.__x_cells, result.__y_cells,
                            result.__z_cells, result.__x_cells*result.__y_cells*result.__z_cells))
        result.__ws_matrix = np.zeros( (result.__x_cells, result.__y_cells, result.__z_cells) )

        for pt in pt_list:
            x_idx = result.getCellIdxX(pt.x())
            y_idx = result.getCellIdxY(pt.y())
            z_idx = result.getCellIdxZ(pt.z())
            if x_idx is None or y_idx is None or z_idx is None:
                print('could not calculate cell idx: {}'.format((x_idx, y_idx, z_idx)))
            result.__ws_matrix[x_idx, y_idx, z_idx] += 1

        max_hits = np.max(result.__ws_matrix)
        print('max hits: {}'.format(max_hits))
        result.__ws_matrix = result.__ws_matrix / max_hits

        return result

    def save(self, parameters_filename, matrix_filename):
        np.save(matrix_filename, self.__ws_matrix)
        ws_params = np.array( [self.__x_min, self.__x_max, self.__y_min, self.__y_max,
                                                self.__z_min, self.__z_max, self.__cell_size] )
        np.save(parameters_filename, ws_params)

    @staticmethod
    def load(parameters_filename=None, matrix_filename=None):
        if parameters_filename is None or matrix_filename is None:
            rospack = rospkg.RosPack()
            ws_data_path = rospack.get_path('velma_kinematics') + '/data/workspace/'
            parameters_filename = ws_data_path + 'lwr_ws_param.npy'
            matrix_filename = ws_data_path + 'lwr_ws.npy'

        result = LWRWorkspace()
        ws_params = np.load(parameters_filename)
        result.__ws_matrix = np.load(matrix_filename)
        result.__x_min, result.__x_max, result.__y_min, result.__y_max,\
                        result.__z_min, result.__z_max, result.__cell_size = ws_params.tolist()
        result.__x_cells = int(math.ceil((result.__x_max - result.__x_min) / result.__cell_size))
        result.__y_cells = int(math.ceil((result.__y_max - result.__y_min) / result.__cell_size))
        result.__z_cells = int(math.ceil((result.__z_max - result.__z_min) / result.__cell_size))

        return result

    def getCellsX(self):
        return self.__x_cells

    def getCellsY(self):
        return self.__y_cells

    def getCellsZ(self):
        return self.__z_cells

    def getCellValue(self, ix, iy=None, iz=None):
        try:
            ix, iy, iz = ix
            assert iy is None
            assert iz is None
        except:
            pass
        if ix is None or ix < 0 or ix >= self.__x_cells:
            return 0.0
        if iy is None or iy < 0 or iy >= self.__y_cells:
            return 0.0
        if iz is None or iz < 0 or iz >= self.__z_cells:
            return 0.0
        return self.__ws_matrix[ix,iy,iz]

    def getCellSize(self):
        return self.__cell_size

class VelmaWorkspace:
    def __init__(self):
        self.__cell_size = None
        self.__x_min = None
        self.__x_max = None
        self.__y_min = None
        self.__y_max = None
        self.__z_min = None
        self.__z_max = None
        self.__x_cells = None
        self.__y_cells = None
        self.__z_cells = None
        self.__ws_matrix = None

    def __getCellIdx(self, v, v_min, cells):
        idx = int(math.floor((v-v_min)/self.__cell_size))
        if idx < 0 or idx >= cells:
            return None
        # else:
        return idx

    def getCellIdxX(self, v):
        return self.__getCellIdx(v, self.__x_min, self.__x_cells)

    def getCellIdxY(self, v):
        return self.__getCellIdx(v, self.__y_min, self.__y_cells)

    def getCellIdxZ(self, v):
        return self.__getCellIdx(v, self.__z_min, self.__z_cells)

    def getCellIdx(self, pt):
        ix = self.getCellIdxX(pt.x())
        iy = self.getCellIdxY(pt.y())
        iz = self.getCellIdxZ(pt.z())
        if ix is None or iy is None or iz is None:
            return None
        # else:
        return (ix, iy, iz)

    def __getCellCenter(self, idx, v_min, cells):
        if idx is None or idx < 0 or idx >= cells:
            return None
        return v_min + (idx+0.5)*self.__cell_size

    def getCellCenterX(self, idx):
        return self.__getCellCenter(idx, self.__x_min, self.__x_cells)

    def getCellCenterY(self, idx):
        return self.__getCellCenter(idx, self.__y_min, self.__y_cells)

    def getCellCenterZ(self, idx):
        return self.__getCellCenter(idx, self.__z_min, self.__z_cells)

    @staticmethod
    def generate():
        result = VelmaWorkspace()

        # Parameters
        samples_count = 20000000
        lim_dist = 0.2
        result.__cell_size = 0.1

        solv = KinematicsSolverVelma()

        q_lim = [ [-2.96, 2.96],
            [-2.09, -0.2, 0.2, 2.09],
            [-2.96, 2.96],
            [-2.095, -0.2, 0.2, 2.095],
            [-2.96, 2.96],
            [-2.09, -0.2, 0.2, 2.09],
            [-2.96, 2.96] ]

        pt_list = []
        x_min = 1000
        x_max = -1000
        y_min = 1000
        y_max = -1000
        z_min = 1000
        z_max = -1000
        for idx in range(samples_count):
            q = []
            for q_idx in range(7):
                q.append( random.uniform(q_lim[q_idx][0]+lim_dist, q_lim[q_idx][-1]-lim_dist) )
            torso_angle = random.uniform( -1.56, 1.56)
            T_B_E = solv.getArmFk( 'right', torso_angle, q )
            pt_list.append(T_B_E.p)
            x_min = min(x_min, T_B_E.p.x())
            x_max = max(x_max, T_B_E.p.x())
            y_min = min(y_min, T_B_E.p.y())
            y_max = max(y_max, T_B_E.p.y())
            z_min = min(z_min, T_B_E.p.z())
            z_max = max(z_max, T_B_E.p.z())
            if idx%1000 == 0:
                print('generated samples: {} / {}'.format(idx, samples_count))

        result.__x_min = x_min
        result.__x_max = x_max
        result.__y_min = y_min
        result.__y_max = y_max
        result.__z_min = z_min
        result.__z_max = z_max

        result.__x_cells = int(math.ceil((result.__x_max - result.__x_min) / result.__cell_size))
        result.__y_cells = int(math.ceil((result.__y_max - result.__y_min) / result.__cell_size))
        result.__z_cells = int(math.ceil((result.__z_max - result.__z_min) / result.__cell_size))

        print('ranges: ({}, {}), ({}, {}), ({}, {})'.format(result.__x_min, result.__x_max,
                            result.__y_min, result.__y_max, result.__z_min, result.__z_max))
        print('cells: {}, {}, {}, total: {}'.format(result.__x_cells, result.__y_cells,
                            result.__z_cells, result.__x_cells*result.__y_cells*result.__z_cells))
        result.__ws_matrix = np.zeros( (result.__x_cells, result.__y_cells, result.__z_cells) )

        for pt in pt_list:
            x_idx = result.getCellIdxX(pt.x())
            y_idx = result.getCellIdxY(pt.y())
            z_idx = result.getCellIdxZ(pt.z())
            if x_idx is None or y_idx is None or z_idx is None:
                print('could not calculate cell idx: {}'.format((x_idx, y_idx, z_idx)))
            result.__ws_matrix[x_idx, y_idx, z_idx] += 1

        max_hits = np.max(result.__ws_matrix)
        print('max hits: {}'.format(max_hits))
        result.__ws_matrix = result.__ws_matrix / max_hits

        return result

    def save(self, parameters_filename, matrix_filename):
        np.save(matrix_filename, self.__ws_matrix)
        ws_params = np.array( [self.__x_min, self.__x_max, self.__y_min, self.__y_max,
                                                self.__z_min, self.__z_max, self.__cell_size] )
        np.save(parameters_filename, ws_params)

    @staticmethod
    def load(parameters_filename=None, matrix_filename=None):
        if parameters_filename is None or matrix_filename is None:
            rospack = rospkg.RosPack()
            ws_data_path = rospack.get_path('velma_kinematics') + '/data/workspace/'
            parameters_filename = ws_data_path + 'velma_ws_param.npy'
            matrix_filename = ws_data_path + 'velma_ws.npy'

        result = VelmaWorkspace()
        ws_params = np.load(parameters_filename)
        result.__ws_matrix = np.load(matrix_filename)
        result.__x_min, result.__x_max, result.__y_min, result.__y_max,\
                        result.__z_min, result.__z_max, result.__cell_size = ws_params.tolist()
        result.__x_cells = int(math.ceil((result.__x_max - result.__x_min) / result.__cell_size))
        result.__y_cells = int(math.ceil((result.__y_max - result.__y_min) / result.__cell_size))
        result.__z_cells = int(math.ceil((result.__z_max - result.__z_min) / result.__cell_size))

        return result

    def getCellsX(self):
        return self.__x_cells

    def getCellsY(self):
        return self.__y_cells

    def getCellsZ(self):
        return self.__z_cells

    def __interpretCellIdx(self, ix, iy, iz):
        try:
            ix, iy, iz = ix
            assert iy is None
            assert iz is None
        except:
            pass
        if ix is None or ix < 0 or ix >= self.__x_cells:
            return None, None, None
        if iy is None or iy < 0 or iy >= self.__y_cells:
            return None, None, None
        if iz is None or iz < 0 or iz >= self.__z_cells:
            return None, None, None
        return ix, iy, iz

    def getCellValueRight(self, ix, iy=None, iz=None):
        ix, iy, iz = self.__interpretCellIdx(ix, iy, iz)
        if ix is None:
            return 0.0
        # else:
        return self.__ws_matrix[ix,iy,iz]

    def getCellValueLeft(self, ix, iy=None, iz=None):
        ix, iy, iz = self.__interpretCellIdx(ix, iy, iz)
        if ix is None:
            return 0.0
        # else:
        return self.__ws_matrix[ix,self.__y_cells-1-iy,iz]

    def getCellValue(self, ix, iy=None, iz=None):
        ix, iy, iz = self.__interpretCellIdx(ix, iy, iz)
        if ix is None:
            return 0.0
        # else:
        return (self.__ws_matrix[ix,iy,iz] + self.__ws_matrix[ix,self.__y_cells-1-iy,iz]) / 2.0

    def getCellValueSide(self, side_str, ix, iy=None, iz=None):
        if side_str == 'left':
            return self.getCellValueLeft(ix, iy, iz)
        elif side_str == 'right':
            return self.getCellValueRight(ix, iy, iz)
        elif side_str == 'any' or side_str == 'both':
            return self.getCellValue(ix, iy, iz)
        # else:
        raise Exception('Wrong value for "side": "{}"'.format(side_str))

    def getCellSize(self):
        return self.__cell_size

NEW CALIBRATION FUNCTIONALITY
------------------------------------------

Calibration system scheme:

          e_self0      e_self1      e_self2      e_selfN
          |            |            |            |
 e_selfC  v0           v1           v2   ...     vn
       |  |            |            |            |
       c0-|e_tern0  c0-|e_tern1  c0-|e_tern2  c0-e_ternN
          |            |            |            |
          i0           i1           i2   ...     in
          |            |            |            |
          e_self0      e_self1      e_self2      e_selfN

where:
c0 - calibration vertex, 6DOF, stored in axis angle format. This represents the 
     transformation from IMU frame to VELO frame.
i* - IMU 6DOF pose, stored in axis angle format.
v* - VELODYNE 6DOF pose, stored in axis angle format.
e_self* - self measurement edge, 6DOF with info matrix. Measures pose of a
          frame.
e_tern* - ternary edge, joining imu, velo and calibration edge. This edge
          represents relation: velo = imu * calibration

Graph file format:
------------------------------------------
Vertices are initialized from edges, no need to put them into system first.

EDGE3SELF:AXISANGLE id x y z aa0 aa1 aa2 m00 m01 m02 m03 m04 m05 m11 m12 m13 m14 m15 m22 m23 m24 m25 m33 m34 m35 m44 m45 m55
id - identifier of the vertex
x y z - position
aa0 aa1 aa2 - rotation in axis angle format
m00 m01 m02 m03 m04 m05
m01 m11 m12 m13 m14 m15
m02 m12 m22 m23 m24 m25
m03 m13 m23 m33 m34 m35 - information matrix, input is in upper triangular format
m04 m13 m24 m34 m44 m45
m05 m15 m25 m35 m45 m55

EDGE3TERNARY:AXISANGLE id0 id1 idC 0 0 0 0 0 0 m00 m01 m02 m03 m04 m05 m11 m12 m13 m14 m15 m22 m23 m24 m25 m33 m34 m35 m44 m45 m55
id0 - id of a vertex "from" (imu)
id1 - id of a vertex "to" (velo)
idC - id of calibration vertex
0 0 0 0 0 0 - measurement, this should be all zero (id0 * idC - id1 should be "zero")

Simple example:
------------------------------------------
Here is a simple example of a system with two measurements

EDGE3SELF:AXISANGLE 0 0 0 0 0 0 0 100 0 0 0 0 0 100 0 0 0 0 100 0 0 0 10000 0 0 10000 0 10000 // imu0
EDGE3SELF:AXISANGLE 1 0 0 1 0 0 0 100 0 0 0 0 0 100 0 0 0 0 100 0 0 0 10000 0 0 10000 0 10000 // imu1
EDGE3SELF:AXISANGLE 2 1 0 0 0 0 0 100 0 0 0 0 0 100 0 0 0 0 100 0 0 0 10000 0 0 10000 0 10000 // velo0
EDGE3SELF:AXISANGLE 3 1 0 1 0 0 0 100 0 0 0 0 0 100 0 0 0 0 100 0 0 0 10000 0 0 10000 0 10000 // velo1
EDGE3SELF:AXISANGLE 4 0.5 0 0 0 0 0 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1 // calibration initialization, low certainty
EDGE3TERNARY:AXISANGLE 0 2 4 0 0 0 0 0 0 100 0 0 0 0 0 100 0 0 0 0 100 0 0 0 10000 0 0 10000 0 10000
EDGE3TERNARY:AXISANGLE 1 3 4 0 0 0 0 0 0 100 0 0 0 0 0 100 0 0 0 0 100 0 0 0 10000 0 0 10000 0 10000
 
Output:
------------------------------------------
output is in solution.txt, on the last line, in axis angle format
best is to use lambda_lm solver e.g.: slam_plus_plus --lambda-lm -i example.graph

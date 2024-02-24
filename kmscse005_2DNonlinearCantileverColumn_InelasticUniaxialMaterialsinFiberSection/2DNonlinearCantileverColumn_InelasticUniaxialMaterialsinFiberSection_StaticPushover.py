import openseespy.opensees as ops
import math

# SET UP ----------------------------------------------------------------------------
ops.wipe()
ops.model('basic', '-ndm', 2, '-ndf', 3)

# define GEOMETRY -------------------------------------------------------------
LCol = 432
Weight = 2000.0
HCol = 60
BCol = 60

# calculated parameters
PCol = Weight
g = 386.4
Mass = PCol / g
ACol = BCol * HCol
IzCol = 1./12. * BCol * HCol ** 3

# nodal coordinates
ops.node(1, 0, 0)
ops.node(2, 0, LCol)

# Boundary Conditions
ops.fix(1, 1, 1, 1)

# nodal masses
ops.mass(2, Mass, 1e-9, 0.0)

# Define ELEMENTS & SECTIONS -------------------------------------------------------------
ColSecTag = 1
coverCol = 5.0
numBarsCol = 5
barAreaCol = 2.25

# MATERIAL parameters
IDconcU = 1
IDreinf = 2
fc = -4.0
Ec = 57 * math.sqrt(abs(fc) * 1000)

# unconfined concrete
fc1U = fc
eps1U = -0.003
fc2U = 0.2 * fc1U
eps2U = -0.01
lambdaU = 0.1
ftU = -0.14 * fc1U
Ets = ftU / 0.002

# steel
Fy = 66.8
Es = 29000.0
Bs = 0.01
R0 = 18
cR1 = 0.925
cR2 = 0.15

ops.uniaxialMaterial('Concrete02', IDconcU, fc1U, eps1U, fc2U, eps2U, lambdaU, ftU, Ets)
ops.uniaxialMaterial('Steel02', IDreinf, Fy, Es, Bs, R0, cR1, cR2)

# FIBER SECTION properties
coverY = HCol / 2.0
coverZ = BCol / 2.0
coreY = coverY - coverCol
coreZ = coverZ - coverCol
nfY = 16
nfZ = 4

ops.section('Fiber', ColSecTag)
ops.patch('quad', IDconcU, nfZ, nfY, -coverY, coverZ, -coverY, -coverZ, coverY, -coverZ, coverY, coverZ)
ops.layer('straight', IDreinf, numBarsCol, barAreaCol, -coreY, coreZ, -coreY, -coreZ)
ops.layer('straight', IDreinf, numBarsCol, barAreaCol, coreY, coreZ, coreY, -coreZ)

# Geometric Transformation
ColTransfTag = 1
ops.geomTransf('Linear', ColTransfTag)

# Element connectivity
numIntgrPts = 5
ops.element('nonlinearBeamColumn', 1, 1, 2, numIntgrPts, ColSecTag, ColTransfTag)

# Define RECORDERS -------------------------------------------------------------
# (Assuming the directory "Data" exists)
ops.recorder('Node', '-file', 'Data/DFree.out', '-time', '-node', 2, '-dof', 1, 2, 3, 'disp') # Displacements of free nodes
ops.recorder('Node', '-file', 'Data/DBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'disp') # Displacements of support nodes
ops.recorder('Node', '-file', 'Data/RBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'reaction') # Support reaction
ops.recorder('Drift', '-file', 'Data/Drift.out', '-time', '-iNode', 1, '-jNode', 2, '-dof', 1, '-perpDirn', 2) # Lateral drift
ops.recorder('Element', '-file', 'Data/FCol.out', '-time', '-ele', 2, 'globalForce') # Element forces -- column
ops.recorder('Element', '-file', 'Data/ForceColSec1.out', '-time', '-ele', 1, 'section', 1, 'force') # Column section forces, axial and moment, node i
ops.recorder('Element', '-file', 'Data/DefoColSec1.out', '-time', '-ele', 1, 'section', 1, 'deformation') # Section deformations, axial and curvature, node i
# Assuming 'numIntgrPts' is defined in your script
numIntgrPts = 5  # Define or replace with the actual number of integration points used in your element
ops.recorder('Element', '-file', 'Data/ForceColSec' + str(numIntgrPts) + '.out', '-time', '-ele', 1, 'section', numIntgrPts, 'force') # Section forces, axial and moment, node j
ops.recorder('Element', '-file', 'Data/DefoColSec' + str(numIntgrPts) + '.out', '-time', '-ele', 1, 'section', numIntgrPts, 'deformation') # Section deformations, axial and curvature, node j

# define GRAVITY -------------------------------------------------------------
ops.pattern('Plain', 1, 'Linear')
ops.load(2, 0, -PCol, 0)

# Gravity-analysis parameters
Tol = 1.0e-8
ops.constraints('Plain')
ops.numberer('Plain')
ops.system('BandGeneral')
ops.test('NormDispIncr', Tol, 6)
ops.algorithm('Newton')
NstepGravity = 10
DGravity = 1.0 / NstepGravity
ops.integrator('LoadControl', DGravity)
ops.analysis('Static')
ops.analyze(NstepGravity)
ops.loadConst('-time', 0.0)

print("Model Built")

# STATIC PUSHOVER ANALYSIS -------------------------------------------------------
IDctrlNode = 2
IDctrlDOF = 1
Dmax = 0.01 * LCol
Dincr = 0.001 * LCol

# create load pattern
Hload = Weight
ops.pattern('Plain', 200, 'Linear')
ops.load(2, Hload, 0.0, 0.0)

# Analysis parameters
ops.constraints('Plain')
ops.numberer('Plain')
ops.system('BandGeneral')
ops.test('EnergyIncr', Tol, 6, 0)
ops.algorithm('Newton')
ops.integrator('DisplacementControl', IDctrlNode, IDctrlDOF, Dincr)
ops.analysis('Static')

# Perform Static Pushover Analysis
Nsteps = int(Dmax / Dincr)
ok = ops.analyze(Nsteps)

# Check for convergence problems
if ok != 0:
    print("Initial analysis did not converge. Attempting modified analysis parameters.")
    ok = 0
    controlDisp = 0.0
    D0 = 0.0

    while True:
        controlDisp = ops.nodeDisp(IDctrlNode, IDctrlDOF)
        Dstep = (controlDisp - D0) / (Dmax - D0)

        if Dstep >= 1.0 or ok == 0:
            break

        TestType = 'NormDispIncr'  # Define the variable TestType
        maxNumIter = 2000
        ok = ops.analyze(1)

        if ok != 0:
            print("Trying Newton with Initial Tangent..")
            ops.test('NormDispIncr', Tol, maxNumIter, 0)
            ops.algorithm('Newton', '-initial')
            ok = ops.analyze(1)
            ops.test(TestType, Tol, maxNumIter, 0)
            ops.algorithm(algorithmType)

        algorithmType = 'Newton'  # Define the variable algorithmType
        if ok != 0:
            print("Trying Broyden..")
            ops.algorithm('Broyden', 8)
            ok = ops.analyze(1)
            ops.algorithm(algorithmType)

        if ok != 0:
            print("Trying NewtonWithLineSearch..")
            ops.algorithm('NewtonLineSearch', 0.8)
            ok = ops.analyze(1)
            ops.algorithm(algorithmType)

# Print a final message if the analysis was successful or not
if ok == 0:
    print("Analysis successful.")
else:
    print("Analysis failed to converge.")

print("DonePushover")

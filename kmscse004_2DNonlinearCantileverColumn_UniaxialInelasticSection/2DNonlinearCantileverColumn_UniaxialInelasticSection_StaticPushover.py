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
ACol = BCol * HCol * 1000
IzCol = 1.0 / 12.0 * BCol * HCol ** 3

# nodal coordinates
ops.node(1, 0, 0)
ops.node(2, 0, LCol)

# Boundary Conditions
ops.fix(1, 1, 1, 1)

# nodal masses
ops.mass(2, Mass, 1e-9, 0.0)

# Define ELEMENTS & SECTIONS -------------------------------------------------------------
ColMatTagFlex = 2
ColMatTagAxial = 3
ColSecTag = 1

# MATERIAL parameters
fc = -4.0
Ec = 57 * math.sqrt(abs(fc) * 1000)

# COLUMN section
EICol = Ec * IzCol
EACol = Ec * ACol
MyCol = 130000
PhiYCol = 0.65e-4
EIColCrack = MyCol / PhiYCol
b = 0.01

ops.uniaxialMaterial('Steel01', ColMatTagFlex, MyCol, EIColCrack, b)
ops.uniaxialMaterial('Elastic', ColMatTagAxial, EACol)
ops.section('Aggregator', ColSecTag, ColMatTagAxial, 'P', ColMatTagFlex, 'Mz')

# Geometric Transformation
ColTransfTag = 1
ops.geomTransf('Linear', ColTransfTag)

# Element connectivity
numIntgrPts = 5
ops.element('nonlinearBeamColumn', 1, 1, 2, numIntgrPts, ColSecTag, ColTransfTag)

# Define RECORDERS -------------------------------------------------------------
ops.recorder('Node', '-file', 'Data/DFree.out', '-time', '-node', 2, '-dof', 1, 2, 3, 'disp') # Displacements of free nodes
ops.recorder('Node', '-file', 'Data/DBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'disp') # Displacements of support nodes
ops.recorder('Node', '-file', 'Data/RBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'reaction') # Support reaction
ops.recorder('Drift', '-file', 'Data/Drift.out', '-time', '-iNode', 1, '-jNode', 2, '-dof', 1, '-perpDirn', 2) # Lateral drift
ops.recorder('Element', '-file', 'Data/FCol.out', '-time', '-ele', 2, 'globalForce') # Element forces -- column
ops.recorder('Element', '-file', 'Data/ForceColSec1.out', '-time', '-ele', 1, 'section', 1, 'force') # Column section forces, axial and moment, node i
ops.recorder('Element', '-file', 'Data/DefoColSec1.out', '-time', '-ele', 1, 'section', 1, 'deformation') # Section deformations, axial and curvature, node i
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
Dmax = 0.05 * LCol
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
# In case of convergence problems
if ok != 0:
    print("Analysis did not converge. Attempting alternative strategies...")
    
    # Resetting the ok flag and initializing variables
    ok = 0
    controlDisp = 0.0
    D0 = 0.0

    # Loop until the displacement reaches the target or analysis converges
    while True:
        controlDisp = ops.nodeDisp(IDctrlNode, IDctrlDOF)
        Dstep = (controlDisp - D0) / (Dmax - D0)
        
        if Dstep >= 1.0 or ok == 0:
            break

        TestType = 'NormDispIncr'  # Define the TestType variable
        maxNumIter = 2000
        ok = ops.analyze(1)

        if ok != 0:
            print("Trying Newton with Initial Tangent...")
            ops.test(TestType, Tol, maxNumIter, 0)
            ops.algorithm('Newton', '-initial')
            ok = ops.analyze(1)
            ops.test(TestType, Tol, maxNumIter, 0)
            ops.algorithm(algorithmType)

        algorithmType = 'Newton'  # Define the algorithmType variable
        if ok != 0:
            print("Trying Broyden...")
            ops.algorithm('Broyden', 8)
            ok = ops.analyze(1)
            ops.algorithm(algorithmType)

        if ok != 0:
            print("Trying NewtonWithLineSearch...")
            ops.algorithm('NewtonLineSearch', 0.8)
            ok = ops.analyze(1)
            ops.algorithm(algorithmType)

# End of convergence issue handling


print("DonePushover")

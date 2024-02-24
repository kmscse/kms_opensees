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
IzCol = 1./12. * BCol * math.pow(HCol, 3)

# nodal coordinates:
ops.node(1, 0, 0)
ops.node(2, 0, LCol)

# Boundary Conditions
ops.fix(1, 1, 1, 1)

# nodal masses:
ops.mass(2, Mass, 1e-9, 0.0)

# Define ELEMENTS & SECTIONS -------------------------------------------------------------
ColMatTagFlex = 2
ColMatTagAxial = 3
ColSecTag = 1

# MATERIAL parameters
fc = -4.
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
# (Specify the file paths for recorder outputs)
ops.recorder('Node', '-file', 'Data/DFree.out', '-time', '-node', 2, '-dof', 1, 2, 3, 'disp') # Displacements of free nodes
ops.recorder('Node', '-file', 'Data/DBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'disp') # Displacements of support nodes
ops.recorder('Node', '-file', 'Data/RBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'reaction') # Support reaction
ops.recorder('Drift', '-file', 'Data/Drift.out', '-time', '-iNode', 1, '-jNode', 2, '-dof', 1, '-perpDirn', 2) # Lateral drift
ops.recorder('Element', '-file', 'Data/FCol.out', '-time', '-ele', 1, 'globalForce') # Element forces -- column
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

# DYNAMIC EQ ANALYSIS --------------------------------------------------------
GMdirection = 1
GMfile = "BM68elc.acc"
GMfact = 1.0

# set up ground-motion-analysis parameters
DtAnalysis = 0.01
TmaxAnalysis = 10.0

# DYNAMIC ANALYSIS PARAMETERS
ops.constraints('Transformation')
ops.numberer('Plain')
ops.system('SparseGeneral', '-piv')
Tol = 1.e-8
maxNumIter = 10
TestType = 'EnergyIncr'
ops.test(TestType, Tol, maxNumIter, 0)
algorithmType = 'ModifiedNewton'
ops.algorithm(algorithmType)
NewmarkGamma = 0.5
NewmarkBeta = 0.25
ops.integrator('Newmark', NewmarkGamma, NewmarkBeta)
ops.analysis('Transient')

# Define damping
xDamp = 0.02
lambda1 = ops.eigen('-fullGenLapack', 1)[0]
omega = math.sqrt(lambda1)
alphaM = 0.0
betaKcurr = 0.0
betaKcomm = 2 * xDamp / omega
betaKinit = 0.0
ops.rayleigh(alphaM, betaKcurr, betaKinit, betaKcomm)

# Perform Dynamic Ground-Motion Analysis
ops.timeSeries('Path', 1, '-dt', DtAnalysis, '-filePath', GMfile, '-factor', GMfact)
ops.pattern('UniformExcitation', 400, GMdirection, '-accel', 1)

Nsteps = int(TmaxAnalysis / DtAnalysis)
ok = ops.analyze(Nsteps, DtAnalysis)

# Handling non-convergence
if ok != 0:
    print("Analysis not successful, attempting modified analysis parameters.")
    controlTime = ops.getTime()
    while controlTime < TmaxAnalysis and ok == 0:
        ok = ops.analyze(1, DtAnalysis)
        controlTime = ops.getTime()
        if ok != 0:
            print("Trying Newton with Initial Tangent..")
            ops.test('NormDispIncr', Tol, 1000, 0)
            ops.algorithm('Newton', '-initial')
            ok = ops.analyze(1, DtAnalysis)
            ops.test(TestType, Tol, maxNumIter, 0)
            ops.algorithm(algorithmType)
        if ok != 0:
            print("Trying Broyden..")
            ops.algorithm('Broyden', 8)
            ok = ops.analyze(1, DtAnalysis)
            ops.algorithm(algorithmType)
        if ok != 0:
            print("Trying NewtonWithLineSearch..")
            ops.algorithm('NewtonLineSearch', 0.8)
            ok = ops.analyze(1, DtAnalysis)
            ops.algorithm(algorithmType)

print("Ground Motion Done. End Time:", ops.getTime())

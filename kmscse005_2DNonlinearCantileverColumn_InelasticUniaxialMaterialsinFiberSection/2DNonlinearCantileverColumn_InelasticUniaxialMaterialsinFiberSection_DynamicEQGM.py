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
numBarsCol = 16
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

# Define RECORDERS (Assuming the directory "Data" exists)
ops.recorder('Node', '-file', 'Data/DFree.out', '-time', '-node', 2, '-dof', 1, 2, 3, 'disp') # Recorder for displacements of free nodes
ops.recorder('Node', '-file', 'Data/DBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'disp') # Recorder for displacements of support nodes
ops.recorder('Node', '-file', 'Data/RBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'reaction') # Recorder for support reaction
ops.recorder('Drift', '-file', 'Data/Drift.out', '-time', '-iNode', 1, '-jNode', 2, '-dof', 1, '-perpDirn', 2) # Recorder for lateral drift
ops.recorder('Element', '-file', 'Data/FCol.out', '-time', '-ele', 1, 'globalForce') # Recorder for element forces -- column
numIntgrPts = 5  # or whatever the actual number is in your script # Assuming numIntgrPts is a defined variable in your script
# Recorders for section forces and deformations at different integration points
for i in range(1, numIntgrPts + 1): 
    ops.recorder('Element', '-file', f'Data/ForceColSec{i}.out', '-time', '-ele', 1, 'section', i, 'force') # Recorder for section forces at each integration point
    ops.recorder('Element', '-file', f'Data/DefoColSec{i}.out', '-time', '-ele', 1, 'section', i, 'deformation') # Recorder for section deformations at each integration point

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

# Analysis parameters
DtAnalysis = 0.01
TmaxAnalysis = 10.0

ops.constraints('Transformation')
ops.numberer('Plain')
ops.system('SparseGeneral', '-piv')
ops.test('EnergyIncr', Tol, 10, 0)
ops.algorithm('ModifiedNewton')
NewmarkGamma = 0.5
NewmarkBeta = 0.25
ops.integrator('Newmark', NewmarkGamma, NewmarkBeta)
ops.analysis('Transient')

# Rayleigh damping
xDamp = 0.02
lambda1 = ops.eigen('-fullGenLapack', 1)[0]
omega = math.sqrt(lambda1)
alphaM = 0.0
betaKcurr = 0.0
betaKcomm = 2.0 * xDamp / omega
betaKinit = 0.0
ops.rayleigh(alphaM, betaKcurr, betaKinit, betaKcomm)

# Ground motion
IDloadTag = 400
dt = 0.01
GMfatt = 1.0
ops.timeSeries('Path', 1, '-dt', dt, '-filePath', GMfile, '-factor', GMfatt)
ops.pattern('UniformExcitation', IDloadTag, GMdirection, '-accel', 1)

Nsteps = int(TmaxAnalysis / DtAnalysis)
ok = ops.analyze(Nsteps, DtAnalysis)

if ops.analyze(1, DtAnalysis) != 0:  # If the analysis was not successful
    # Change some analysis parameters to achieve convergence
    # Performance is slower inside this loop
    # Time-controlled analysis
    ok = 0
    controlTime = ops.getTime()

    while controlTime < TmaxAnalysis and ok == 0:
        ok = ops.analyze(1, DtAnalysis)
        controlTime = ops.getTime()

        maxNumIter = 10  # Define the variable maxNumIter

        algorithmType = 'ModifiedNewton'  # Define the variable algorithmType

        if ok != 0:
            print("Trying Newton with Initial Tangent ..")
            TestType = 'NormDispIncr'  # Define the variable TestType
            ops.test(TestType, Tol, maxNumIter, 0)
            ops.algorithm('Newton', '-initial')
            ok = ops.analyze(1, DtAnalysis)
            ops.test(TestType, Tol, maxNumIter, 0)  # Use the defined variable TestType
            ops.algorithm(algorithmType)

        if ok != 0:
            print("Trying Broyden ..")
            ops.algorithm('Broyden', 8)
            ok = ops.analyze(1, DtAnalysis)
            ops.algorithm(algorithmType)

        if ok != 0:
            print("Trying NewtonWithLineSearch ..")
            ops.algorithm('NewtonLineSearch', .8)
            ok = ops.analyze(1, DtAnalysis)
            ops.algorithm(algorithmType)

print("Ground Motion Done. End Time:", ops.getTime())
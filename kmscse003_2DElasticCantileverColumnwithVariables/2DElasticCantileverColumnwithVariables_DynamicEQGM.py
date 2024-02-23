import openseespy.opensees as ops
import math

# SET UP ----------------------------------------------------------------------------
ops.wipe()  # Clear memory of all past model definitions
ops.model('basic', '-ndm', 2, '-ndf', 3)  # Define the model builder

# define GEOMETRY -------------------------------------------------------------
LCol = 432  # Column length
Weight = 2000  # Superstructure weight
HCol = 60  # Column Depth
BCol = 60  # Column Width

# Calculated parameters
PCol = Weight  # Nodal dead-load weight per column
g = 386.4  # Gravity
Mass = PCol / g  # Nodal mass
ACol = BCol * HCol * 1000  # Cross-sectional area, make stiff
IzCol = 1./12. * BCol * HCol**3  # Column moment of inertia

# Nodal coordinates
ops.node(1, 0, 0)  # Node 1 at (0, 0)
ops.node(2, 0, LCol)  # Node 2 at (0, LCol)

# Boundary Conditions
ops.fix(1, 1, 1, 1)  # Node 1: Fixed in all directions

# Nodal masses
ops.mass(2, Mass, 1e-9, 0.)  # Node 2 mass

# Define ELEMENTS -------------------------------------------------------------
fc = -4.  # Concrete Compressive Strength
Ec = 57 * math.sqrt(abs(fc) * 1000)  # Concrete Elastic Modulus

ColTransfTag = 1
ops.geomTransf('Linear', ColTransfTag)

ops.element('elasticBeamColumn', 1, 1, 2, ACol, Ec, IzCol, ColTransfTag)

# Define RECORDERS -------------------------------------------------------------
# (Specify the file paths for recorder outputs)
ops.recorder('Node', '-file', 'Data/DFree.out', '-time', '-node', 2, '-dof', 1, 2, 3, 'disp')  # Displacements of free nodes
ops.recorder('Node', '-file', 'Data/DBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'disp')  # Displacements of support nodes
ops.recorder('Node', '-file', 'Data/RBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'reaction')  # Support reaction
ops.recorder('Drift', '-file', 'Data/Drift.out', '-time', '-iNode', 1, '-jNode', 2, '-dof', 1, '-perpDirn', 2)  # Lateral drift
ops.recorder('Element', '-file', 'Data/FCol.out', '-time', '-ele', 1, 'globalForce')  # Element forces -- column

# define GRAVITY -------------------------------------------------------------
ops.pattern('Plain', 1, 'Linear')
ops.load(2, 0, -PCol, 0)

# Apply gravity load
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

# Set up ground-motion-analysis parameters
DtAnalysis = 0.01
TmaxAnalysis = 10.0

# Analysis Parameters
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

# Define ground motion
ops.timeSeries('Path', 1, '-dt', DtAnalysis, '-filePath', GMfile, '-factor', GMfact)
ops.pattern('UniformExcitation', 400, GMdirection, '-accel', 1)

# Perform analysis
Nsteps = int(TmaxAnalysis / DtAnalysis)
ok = ops.analyze(Nsteps, DtAnalysis)

# In case of convergence problems
if ok != 0:
    # Modify analysis parameters for convergence
    controlTime = ops.getTime()
    while controlTime < TmaxAnalysis and ok == 0:
        ok = ops.analyze(1, DtAnalysis)
        controlTime = ops.getTime()
        # Additional convergence strategies (e.g., Newton with Initial Tangent) can be implemented here

print("Ground Motion Done. End Time:", ops.getTime())

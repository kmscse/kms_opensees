import openseespy.opensees as ops

# SET UP ----------------------------------------------------------------------------
ops.wipe()  # Clear OpenSees model
ops.model('basic', '-ndm', 2, '-ndf', 3)  # Define the model builder, ndm=#dimension, ndf=#dofs

# define GEOMETRY -------------------------------------------------------------
LCol = 432  # column length
Weight = 2000  # superstructure weight
# define section geometry
HCol = 60  # Column Depth
BCol = 60  # Column Width

# calculated parameters
PCol = Weight  # nodal dead-load weight per column
g = 386.4  # gravity
Mass = PCol / g  # nodal mass
# calculated geometry parameters
ACol = BCol * HCol * 1000  # cross-sectional area, make stiff
IzCol = BCol * HCol ** 3 / 12  # Column moment of inertia

# nodal coordinates:
ops.node(1, 0, 0)  # Node 1 at (0, 0)
ops.node(2, 0, LCol)  # Node 2 at (0, LCol)

# Single point constraints -- Boundary Conditions
ops.fix(1, 1, 1, 1)  # Node 1: Fixed in all directions (DX, DY, RZ)

# nodal masses:
ops.mass(2, Mass, 1e-9, 0.)  # Node 2 mass

# Define ELEMENTS -------------------------------------------------------------
# Material parameters
fc = -4  # CONCRETE Compressive Strength
Ec = 57 * (abs(fc) * 1000) ** 0.5  # Concrete Elastic Modulus

# define geometric transformation
ColTransfTag = 1
ops.geomTransf('Linear', ColTransfTag)

# element connectivity:
ops.element('elasticBeamColumn', 1, 1, 2, ACol, Ec, IzCol, ColTransfTag)

# Define RECORDERS -------------------------------------------------------------
# (Specify the file paths for recorder outputs)
ops.recorder('Node', '-file', 'Data/DFree.out', '-time', '-node', 2, '-dof', 1, 2, 3, 'disp')  # displacements of free nodes
ops.recorder('Node', '-file', 'Data/DBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'disp')  # displacements of support nodes
ops.recorder('Node', '-file', 'Data/RBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'reaction')  # support reaction
ops.recorder('Drift', '-file', 'Data/Drift.out', '-time', '-iNode', 1, '-jNode', 2, '-dof', 1, '-perpDirn', 2)  # lateral drift
ops.recorder('Element', '-file', 'Data/FCol.out', '-time', '-ele', 1, 'globalForce')  # element forces -- column

# define GRAVITY -------------------------------------------------------------
ops.pattern('Plain', 1, 'Linear')  # Gravity load pattern
ops.load(2, 0, -PCol, 0)  # Load on node 2

# apply gravity load
Tol = 1.0e-8  # convergence tolerance for test
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

# STATIC PUSHOVER ANALYSIS --------------------------------------------------------------------------------------------------
IDctrlNode = 2
IDctrlDOF = 1
Dmax = 0.01 * LCol
Dincr = 0.001 * LCol

# create load pattern for lateral pushover load
Hload = Weight
ops.pattern('Plain', 200, 'Linear')
ops.load(2, Hload, 0.0, 0.0)

# Set up analysis parameters
ops.constraints('Plain')
ops.numberer('Plain')
ops.system('BandGeneral')
Tol = 1.e-8
maxNumIter = 6
printFlag = 0
TestType = 'EnergyIncr'
ops.test(TestType, Tol, maxNumIter, printFlag)
algorithmType = 'Newton'
ops.algorithm(algorithmType)
ops.integrator('DisplacementControl', IDctrlNode, IDctrlDOF, Dincr)
ops.analysis('Static')

# Perform Static Pushover Analysis
Nsteps = int(Dmax / Dincr)
ok = ops.analyze(Nsteps)

# In case of convergence problems
if ok != 0:
    # Change some analysis parameters to achieve convergence
    controlDisp = 0.0  # start from zero
    D0 = 0.0  # start from zero
    Dstep = (controlDisp - D0) / (Dmax - D0)
    while Dstep < 1.0 and ok == 0:
        controlDisp = ops.nodeDisp(IDctrlNode, IDctrlDOF)
        Dstep = (controlDisp - D0) / (Dmax - D0)
        ok = ops.analyze(1)
        if ok != 0:
            print("Trying Newton with Initial Tangent ..")
            ops.test('NormDispIncr', Tol, 2000, 0)
            ops.algorithm('Newton', '-initial')
            ok = ops.analyze(1)
            ops.test(TestType, Tol, maxNumIter, 0)
            ops.algorithm(algorithmType)
        if ok != 0:
            print("Trying Broyden ..")
            ops.algorithm('Broyden', 8)
            ok = ops.analyze(1)
            ops.algorithm(algorithmType)
        if ok != 0:
            print("Trying NewtonWithLineSearch ..")
            ops.algorithm('NewtonLineSearch', 0.8)
            ok = ops.analyze(1)
            ops.algorithm(algorithmType)

print("DonePushover")

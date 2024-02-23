import openseespy.opensees as ops

# SET UP ----------------------------------------------------------------------------
ops.wipe()  # clear opensees model
ops.model('basic', '-ndm', 2, '-ndf', 3)  # 2 dimensions, 3 dof per node

# define GEOMETRY -------------------------------------------------------------
ops.node(1, 0, 0)  # node#, X, Y
ops.node(2, 0, 432)

# Single point constraints -- Boundary Conditions
ops.fix(1, 1, 1, 1)  # node DX DY RZ

# nodal masses:
ops.mass(2, 5.18, 1.e-9, 0.)  # node#, Mx, My, Mz, Mass=Weight/g.

# Define ELEMENTS -------------------------------------------------------------
# define geometric transformation
ops.geomTransf('Linear', 1)  # associate a tag to transformation

# connectivity
ops.element('elasticBeamColumn', 1, 1, 2, 3600000000, 4227, 1080000, 1)  # element elasticBeamColumn $eleTag $iNode $jNode $A $E $Iz $transfTag

# Define RECORDERS -------------------------------------------------------------
ops.recorder('Node', '-file', 'data/DFree.out', '-time', '-node', 2, '-dof', 1, 2, 3, 'disp')  # displacements of free nodes
ops.recorder('Node', '-file', 'data/DBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'disp')  # displacements of support nodes
ops.recorder('Node', '-file', 'data/RBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'reaction')  # support reaction
ops.recorder('Drift', '-file', 'data/Drift.out', '-time', '-iNode', 1, '-jNode', 2, '-dof', 1, '-perpDirn', 2)  # lateral drift
ops.recorder('Element', '-file', 'data/FCol.out', '-time', '-ele', 1, 'globalForce')  # element forces -- column
ops.recorder('Element', '-file', 'data/DCol.out', '-time', '-ele', 1, 'deformation')  # element deformations -- column

# define GRAVITY -------------------------------------------------------------
ops.pattern('Plain', 1, 'Linear')  # load pattern
ops.load(2, 0, -2000, 0)  # node#, FX, FY, MZ -- superstructure-weight

ops.constraints('Plain')  # how it handles boundary conditions
ops.numberer('Plain')  # renumber dof's to minimize band-width
ops.system('BandGeneral')  # how to store and solve the system of equations in the analysis
ops.test('NormDispIncr', 1.0e-8, 6)  # determine if convergence has been achieved
ops.algorithm('Newton')  # use Newton's solution algorithm
ops.integrator('LoadControl', 0.1)  # determine the next time step for an analysis
ops.analysis('Static')  # define type of analysis static or transient
ops.analyze(10)  # perform gravity analysis
ops.loadConst('-time', 0.0)  # hold gravity constant and restart time

# DYNAMIC ground-motion analysis -------------------------------------------------------------
ops.wipeAnalysis()  # clear previously-defined analysis parameters
ops.constraints('Plain')  # how it handles boundary conditions
ops.numberer('Plain')  # renumber dof's to minimize band-width
ops.system('BandGeneral')  # how to store and solve the system of equations in the analysis
ops.test('NormDispIncr', 1.0e-8, 10)  # determine if convergence has been achieved
ops.algorithm('Newton')  # use Newton's solution algorithm
ops.integrator('Newmark', 0.5, 0.25)  # determine the next time step for an analysis
ops.analysis('Transient')  # define type of analysis: time-dependent

# apply 1000 0.02-sec time steps in analysis
ops.analyze(1000, 0.02)

print("Done!")

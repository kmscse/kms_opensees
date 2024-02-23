import openseespy.opensees as ops

# SET UP ----------------------------------------------------------------------------
ops.wipe()                        # clear OpenSees model
ops.model('basic', '-ndm', 2, '-ndf', 3)  # 2 dimensions, 3 degrees of freedom per node

# define GEOMETRY -------------------------------------------------------------
ops.node(1, 0, 0)   # node 1 at (0, 0)
ops.node(2, 0, 432) # node 2 at (0, 432)

# Single point constraints -- Boundary Conditions
ops.fix(1, 1, 1, 1)  # Node 1: Fix in all directions (DX, DY, RZ)

# nodal masses:
ops.mass(2, 5.18, 0., 0.)  # node 2 mass

# Define ELEMENTS -------------------------------------------------------------
# define geometric transformation
ops.geomTransf('Linear', 1)  # Linear geometric transformation, tag 1

# define elements
ops.element('elasticBeamColumn', 1, 1, 2, 3600000000, 4227, 1080000, 1)

# Define RECORDERS -------------------------------------------------------------
ops.recorder('Node', '-file', 'Data/DFree.out', '-time', '-node', 2, '-dof', 1, 2, 3, 'disp')
ops.recorder('Node', '-file', 'Data/DBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'disp')
ops.recorder('Node', '-file', 'Data/RBase.out', '-time', '-node', 1, '-dof', 1, 2, 3, 'reaction')
ops.recorder('Drift', '-file', 'Data/Drift.out', '-time', '-iNode', 1, '-jNode', 2, '-dof', 1, '-perpDirn', 2)
ops.recorder('Element', '-file', 'Data/FCol.out', '-time', '-ele', 1, 'globalForce')
ops.recorder('Element', '-file', 'Data/DCol.out', '-time', '-ele', 1, 'deformation')

# define GRAVITY -------------------------------------------------------------
ops.pattern('Plain', 1, 'Linear')  # Gravity load pattern
ops.load(2, 0., -2000., 0.)        # Load on node 2

ops.constraints('Plain')
ops.numberer('Plain')
ops.system('BandGeneral')
ops.test('NormDispIncr', 1.0e-8, 6)
ops.algorithm('Newton')
ops.integrator('LoadControl', 0.1)  # Load control for gravity
ops.analysis('Static')
ops.analyze(10)                    # Perform gravity analysis
ops.loadConst('-time', 0.0)        # Hold gravity loads constant

# define LATERAL load -------------------------------------------------------------
ops.pattern('Plain', 2, 'Linear')  # Lateral load pattern
ops.load(2, 2000., 0.0, 0.0)       # Lateral load on node 2

# pushover analysis
ops.integrator('DisplacementControl', 2, 1, 0.1)  # Displacement control
ops.analyze(1000)                  # Apply pushover analysis

print("Done!")

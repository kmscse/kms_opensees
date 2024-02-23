import openseespy.opensees as ops

# SET UP ----------------------------------------------------------------------------
ops.wipe()  # Clear OpenSees model
ops.model('basic', '-ndm', 2, '-ndf', 3)  # 2 dimensions, 3 degrees of freedom per node

# define GEOMETRY -------------------------------------------------------------
ops.node(1, 0, 0)   # Node 1 at (0, 0)
ops.node(2, 504, 0) # Node 2 at (504, 0)
ops.node(3, 0, 432) # Node 3 at (0, 432)
ops.node(4, 504, 432) # Node 4 at (504, 432)

# Single point constraints -- Boundary Conditions
ops.fix(1, 1, 1, 1)  # Node 1: Fixed in all directions (DX, DY, RZ)
ops.fix(2, 1, 1, 1)  # Node 2: Fixed in all directions (DX, DY, RZ)
ops.fix(3, 0, 0, 0)  # Node 3: Free in all directions
ops.fix(4, 0, 0, 0)  # Node 4: Free in all directions

# nodal masses:
ops.mass(3, 5.18, 0., 0.)  # Node 3 mass
ops.mass(4, 5.18, 0., 0.)  # Node 4 mass

# Define ELEMENTS -------------------------------------------------------------
ops.geomTransf('Linear', 1)  # Linear geometric transformation, tag 1

# define elements
ops.element('elasticBeamColumn', 1, 1, 3, 3600000000, 4227, 1080000, 1)  # element 1
ops.element('elasticBeamColumn', 2, 2, 4, 3600000000, 4227, 1080000, 1)  # element 2
ops.element('elasticBeamColumn', 3, 3, 4, 5760000000, 4227, 4423680, 1)  # element 3

# Define RECORDERS -------------------------------------------------------------
ops.recorder('Node', '-file', 'Data/DFree.out', '-time', '-node', 3, 4, '-dof', 1, 2, 3, 'disp')
ops.recorder('Node', '-file', 'Data/DBase.out', '-time', '-node', 1, 2, '-dof', 1, 2, 3, 'disp')
ops.recorder('Node', '-file', 'Data/RBase.out', '-time', '-node', 1, 2, '-dof', 1, 2, 3, 'reaction')
ops.recorder('Drift', '-file', 'Data/Drift.out', '-time', '-iNode', 1, 2, '-jNode', 3, 4, '-dof', 1, '-perpDirn', 2)
ops.recorder('Element', '-file', 'Data/FCol.out', '-time', '-ele', 1, 2, 'globalForce')
ops.recorder('Element', '-file', 'Data/FBeam.out', '-time', '-ele', 3, 'globalForce')

# define GRAVITY -------------------------------------------------------------
ops.pattern('Plain', 1, 'Linear')  # Gravity load pattern
ops.eleLoad('-ele', 3, '-type', '-beamUniform', -7.94)  # Distributed load on beam

ops.constraints('Plain')
ops.numberer('Plain')
ops.system('BandGeneral')
ops.test('NormDispIncr', 1.0e-8, 6)
ops.algorithm('Newton')
ops.integrator('LoadControl', 0.1)  # Load control for gravity
ops.analysis('Static')
ops.analyze(10)  # Perform gravity analysis
ops.loadConst('-time', 0.0)  # Hold gravity loads constant

# define LATERAL load -------------------------------------------------------------
ops.pattern('Plain', 2, 'Linear')  # Lateral load pattern
ops.load(3, 2000., 0.0, 0.0)  # Lateral load on node 3
ops.load(4, 2000., 0.0, 0.0)  # Lateral load on node 4

# pushover analysis
ops.integrator('DisplacementControl', 3, 1, 0.1)  # Displacement control for pushover analysis
ops.analyze(100)  # Apply pushover analysis

print("Done!")

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

# Define elements
ops.element('elasticBeamColumn', 1, 1, 3, 3600000000, 4227, 1080000, 1)  # element 1
ops.element('elasticBeamColumn', 2, 2, 4, 3600000000, 4227, 1080000, 1)  # element 2
ops.element('elasticBeamColumn', 3, 3, 4, 5760000000, 4227, 4423680, 1)  # element 3

# Define RECORDERS -------------------------------------------------------------
# (Specify the file paths for recorder outputs)

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

# DYNAMIC ground-motion analysis -------------------------------------------------------------
# Define the time series
ops.timeSeries('Path', 1, '-dt', 0.01, '-filePath', 'BM68elc.acc', '-factor', 1)
ops.pattern('UniformExcitation', 2, 1, '-accel', 1)  # Define where and how acceleration is applied

# Eigenvalue analysis for determining damping
lambda1 = ops.eigen('-fullGenLapack', 1)[0]
omega1 = (lambda1 ** 0.5)
xi = 0.02  # Damping ratio
alphaM = 0.0
betaK = 0.0
betaK0 = 0.0
betaKc = 2 * xi / omega1
ops.rayleigh(alphaM, betaK, betaK0, betaKc)  # Rayleigh damping

# Create the analysis
ops.wipeAnalysis()
ops.constraints('Plain')
ops.numberer('Plain')
ops.system('BandGeneral')
ops.test('NormDispIncr', 1.0e-8, 10)
ops.algorithm('Newton')
ops.integrator('Newmark', 0.5, 0.25)
ops.analysis('Transient')

ops.analyze(1000, 0.02)  # Apply 1000 0.02-sec time steps in analysis

print("Done!")

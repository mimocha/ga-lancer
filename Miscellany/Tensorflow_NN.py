# 12/19/17 Neural Net Control for Simulation Car
#	Weights 10 | Biases 2 | Hidden Layers 0 | Hidden Nodes 0

import tensorflow as tf

# Base Model
x = tf.placeholder(tf.float32, [5, 1])
W = tf.Variable(tf.random_normal([2, 5]))
b = tf.Variable(tf.random_normal([2, 1]))
y = tf.placeholder(tf.float32, [2, 1])
model = tf.matmul(W, x) + b

# Loss Value
y_ = tf.reduce_sum(tf.square(model - y))
optimizer = tf.train.GradientDescentOptimizer(0.01)
train = optimizer.minimize(y_)

# Training Data
	# Python Matrix Reference -> stackoverflow.com/questions/6667201
	# Matrix = [z][y][x] => [Set No.][Height][Width]
	# => [[[0 for x in range()] for y in range()] for z in range()]

# Training Set -> How many data set to train with
s_t = 20
# Initializing Multidimension Array
x_t = [[[0 for ax in range(1)] for ay in range(5)] for az in range(s_t)]
y_t = [[[0 for bx in range(1)] for by in range(2)] for bz in range(s_t)]

# Crude manual matrix definition
# Need data generation algorithm
x_t[0] = [[1], [1], [1], [1], [1]]
x_t[1] = [[0], [1], [1], [1], [1]]
x_t[2] = [[1], [1], [1], [1], [0]]
x_t[3] = [[0.5], [0.5], [1], [1], [1]]
x_t[4] = [[1], [1], [1], [0.5], [0.5]]
x_t[5] = [[0], [1], [1], [1], [0]]
x_t[6] = [[0.5], [0.5], [1], [0.5], [0.5]]
x_t[7] = [[0], [0], [1], [0], [0]]
x_t[8] = [[0], [0.5], [0.1], [0.5], [0]]
x_t[9] = [[0.5], [0.5], [0.5], [0.5], [0.5]]
x_t[10] = [[1], [0], [0], [0], [1]]
x_t[11] = [[0], [0], [0], [0], [1]]
x_t[12] = [[1], [0], [0], [0], [0]]
x_t[13] = [[1], [0.5], [0.3], [0.5], [1]]
x_t[14] = [[1], [1], [1], [0.5], [0]]
x_t[15] = [[0], [0.5], [1], [1], [1]]
x_t[16] = [[0], [0], [0], [0], [0]]
x_t[17] = [[1], [0], [1], [0], [1]]
x_t[18] = [[0.5], [0.5], [0], [0], [0]]
x_t[19] = [[0], [0], [0], [0.5], [0.5]]

y_t[0] = [[0], [1]]
y_t[1] = [[1], [0.7]]
y_t[2] = [[-1], [0.7]]
y_t[3] = [[1], [1]]
y_t[4] = [[-1], [1]]
y_t[5] = [[0], [0.5]]
y_t[6] = [[0], [0.5]]
y_t[7] = [[0], [0.1]]
y_t[8] = [[0], [0.1]]
y_t[9] = [[0], [0.5]]
y_t[10] = [[0], [-1]]
y_t[11] = [[-1], [-1]]
y_t[12] = [[1], [-1]]
y_t[13] = [[0], [0.1]]
y_t[14] = [[1], [0.7]]
y_t[15] = [[-1], [0.7]]
y_t[16] = [[0], [-1]]
y_t[17] = [[0], [0.5]]
y_t[18] = [[1], [-0.3]]
y_t[19] = [[-1], [-0.3]]


for k in range(1):
	# Training Loop
	init = tf.global_variables_initializer()
	sess = tf.Session()
	sess.run(init)

	# How many loops to do
	MAX = 10000
	for i in range(MAX):
		for j in range(s_t):
			sess.run(train, {x: x_t[j], y: y_t[j]})
		#print(i, sess.run(W), sess.run(b))
		prog = i/MAX*100
		print("Training Percentage: %s"%prog)


	fin_W, fin_b, fin_l = sess.run([W, b, y_], {x: x_t[0], y: y_t[0]})
	print("----- Results Loop %s -----"%k)
	print("W: %s"%fin_W)
	print("b: %s"%fin_b)
	print("loss: %s"%fin_l)
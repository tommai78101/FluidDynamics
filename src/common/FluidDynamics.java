package common;

import java.util.Arrays;

public class FluidDynamics {
	public static final int N = 256;
	public static final int Iterations = 4;
	public static final int Scale = 3;
	public static final float Friction = (1f - 0.003f);

	public float deltaTime;
	public float diffusion;
	public float viscosity;

	public float[] size;
	public float[] density;
	public float[] xVelocity;
	public float[] yVelocity;
	public float[] xPreviousVelocity;
	public float[] yPreviousVelocity;

	public FluidDynamics(float deltaTime, float diffusion, float viscocity) {
		this.deltaTime = deltaTime;
		this.diffusion = diffusion;
		this.viscosity = viscocity;
		this.deltaTime = 0f;

		this.size = new float[FluidDynamics.N * FluidDynamics.N];
		this.density = new float[FluidDynamics.N * FluidDynamics.N];
		this.xVelocity = new float[FluidDynamics.N * FluidDynamics.N];
		this.yVelocity = new float[FluidDynamics.N * FluidDynamics.N];
		this.xPreviousVelocity = new float[FluidDynamics.N * FluidDynamics.N];
		this.yPreviousVelocity = new float[FluidDynamics.N * FluidDynamics.N];
	}

	public static void advect(int boundary, float[] density, float[] densityPrevious, float[] xVelo, float[] yVelo, float deltaTime) {
		float i0, i1, j0, j1;
		float deltaTimeX = deltaTime * (FluidDynamics.N - 2);
		float deltaTimeY = deltaTime * (FluidDynamics.N - 2);

		float s0, s1, t0, t1;
		float temp1, temp2, x, y;

		float nFloat = FluidDynamics.N;
		float iFloat, jFloat;
		int i, j;

		for (j = 1, jFloat = 1.0f; j < FluidDynamics.N - 1; j++, jFloat += 1.0f) {
			for (i = 1, iFloat = 1.0f; i < FluidDynamics.N - 1; i++, iFloat += 1.0f) {
				temp1 = deltaTimeX * xVelo[FluidDynamics.getIndex(i, j)];
				temp2 = deltaTimeY * yVelo[FluidDynamics.getIndex(i, j)];
				x = iFloat - temp1;
				y = jFloat - temp2;

				if (x < 0.5f)
					x = 0.5f;
				if (x > nFloat + 0.5f)
					x = nFloat + 0.5f;
				i0 = (float) Math.floor(x);
				i1 = i0 + 1.0f;

				if (y < 0.5f)
					y = 0.5f;
				if (y > nFloat + 0.5f)
					y = nFloat + 0.5f;
				j0 = (float) Math.floor(y);
				j1 = j0 + 1.0f;

				s1 = x - i0;
				s0 = 1.0f - s1;

				t1 = y - j0;
				t0 = 1.0f - t1;

				int i0i = (int) i0;
				int i1i = (int) i1;
				int j0i = (int) j0;
				int j1i = (int) j1;

				// @formatter:off
				density[FluidDynamics.getIndex(i, j)] =
					s0 * (t0 * densityPrevious[FluidDynamics.getIndex(i0i, j0i)] + t1 * densityPrevious[FluidDynamics.getIndex(i0i, j1i)]) +
					s1 * (t0 * densityPrevious[FluidDynamics.getIndex(i1i, j0i)] + t1 * densityPrevious[FluidDynamics.getIndex(i1i, j1i)]);
				// @formatter:on
			}
		}

		FluidDynamics.setBoundary(boundary, density);
	}

	public static float constrain(float value, float min, float max) {
		if (value < min)
			value = min;
		if (value > max)
			value = max;
		return value;
	}

	public static int constrain(int value, int min, int max) {
		if (value < min)
			value = min;
		if (value > max)
			value = max;
		return value;
	}

	public static void diffuse(int boundary, float[] x, float[] xPrevious, float diffusion, float deltaTime) {
		// Removing the outer-most edges of squares (left-, right-, top- and bottom-most).
		float newDiffusion = deltaTime * diffusion * ((FluidDynamics.N - 2) * (FluidDynamics.N - 2));
		FluidDynamics.linearSolve(boundary, x, xPrevious, newDiffusion, 1 + 6 * newDiffusion);
	}

	public static void friction(float[] velocity, int index, float constant) {
		velocity[index] *= constant;
		if (velocity[index] < constant) {
			velocity[index] = 0f;
		}
	}

	public static int getIndex(int x, int y) {
		x = FluidDynamics.constrain(x, 0, FluidDynamics.N - 1);
		y = FluidDynamics.constrain(y, 0, FluidDynamics.N - 1);
		return x + y * FluidDynamics.N;
	}

	public static void linearSolve(int boundary, float[] x, float[] xPrevious, float newDiffusion, float constant) {
		float constantReciprocal = 1.0f / constant;
		for (int k = 0; k < FluidDynamics.Iterations; k++) {
			for (int j = 1; j < FluidDynamics.N - 1; j++) {
				for (int i = 1; i < FluidDynamics.N - 1; i++) {
					x[FluidDynamics.getIndex(i, j)] = (xPrevious[FluidDynamics.getIndex(i, j)] +
						newDiffusion * (x[FluidDynamics.getIndex(i + 1, j)] +
							x[FluidDynamics.getIndex(i - 1, j)] +
							x[FluidDynamics.getIndex(i, j + 1)] +
							x[FluidDynamics.getIndex(i, j - 1)]))
						* constantReciprocal;
				}
			}
		}

		FluidDynamics.setBoundary(boundary, x);
	}

	public static void project(float[] xVelo, float[] yVelo, float[] p, float[] div) {
		for (int j = 1; j < FluidDynamics.N - 1; j++) {
			for (int i = 1; i < FluidDynamics.N - 1; i++) {
				div[FluidDynamics.getIndex(i, j)] = -0.5f * (xVelo[FluidDynamics.getIndex(i + 1, j)] - xVelo[FluidDynamics.getIndex(i - 1, j)] +
					yVelo[FluidDynamics.getIndex(i, j + 1)] - yVelo[FluidDynamics.getIndex(i, j - 1)]) / FluidDynamics.N;
				p[FluidDynamics.getIndex(i, j)] = 0;
			}
		}
		FluidDynamics.setBoundary(0, div);
		FluidDynamics.setBoundary(0, p);
		FluidDynamics.linearSolve(0, p, div, 1, 6);
		for (int j = 1; j < FluidDynamics.N - 1; j++) {
			for (int i = 1; i < FluidDynamics.N - 1; i++) {
				xVelo[FluidDynamics.getIndex(i, j)] -= 0.5f * (p[FluidDynamics.getIndex(i + 1, j)] - p[FluidDynamics.getIndex(i - 1, j)]) * FluidDynamics.N;
				yVelo[FluidDynamics.getIndex(i, j)] -= 0.5f * (p[FluidDynamics.getIndex(i, j + 1)] - p[FluidDynamics.getIndex(i, j - 1)]) * FluidDynamics.N;
			}
		}
		FluidDynamics.setBoundary(1, xVelo);
		FluidDynamics.setBoundary(2, yVelo);
	}

	public static void setBoundary(int boundary, float[] data) {
		for (int i = 1; i < FluidDynamics.N - 1; i++) {
			data[FluidDynamics.getIndex(i, 0)] = boundary == 2 ? -data[FluidDynamics.getIndex(i, 1)] : data[FluidDynamics.getIndex(i, 1)];
			data[FluidDynamics.getIndex(i, FluidDynamics.N - 1)] = boundary == 2 ? -data[FluidDynamics.getIndex(i, FluidDynamics.N - 2)] : data[FluidDynamics.getIndex(i, FluidDynamics.N - 2)];
		}
		for (int j = 1; j < FluidDynamics.N - 1; j++) {
			data[FluidDynamics.getIndex(0, j)] = boundary == 1 ? -data[FluidDynamics.getIndex(1, j)] : data[FluidDynamics.getIndex(1, j)];
			data[FluidDynamics.getIndex(FluidDynamics.N - 1, j)] = boundary == 1 ? -data[FluidDynamics.getIndex(FluidDynamics.N - 2, j)] : data[FluidDynamics.getIndex(FluidDynamics.N - 2, j)];
		}
		data[FluidDynamics.getIndex(0, 0)] = 0.5f * (data[FluidDynamics.getIndex(1, 0)] + data[FluidDynamics.getIndex(0, 1)]);
		data[FluidDynamics.getIndex(0, FluidDynamics.N - 1)] = 0.5f * (data[FluidDynamics.getIndex(1, FluidDynamics.N - 1)] + data[FluidDynamics.getIndex(0, FluidDynamics.N - 2)]);
		data[FluidDynamics.getIndex(FluidDynamics.N - 1, 0)] = 0.5f * (data[FluidDynamics.getIndex(FluidDynamics.N - 2, 0)] + data[FluidDynamics.getIndex(FluidDynamics.N - 1, 1)]);
		data[FluidDynamics.getIndex(FluidDynamics.N - 1, FluidDynamics.N - 1)] = 0.5f * (data[FluidDynamics.getIndex(FluidDynamics.N - 2, FluidDynamics.N - 1)] + data[FluidDynamics.getIndex(FluidDynamics.N - 1, FluidDynamics.N - 2)]);
	}

	public void addDensity(int x, int y, float amount) {
		int index = FluidDynamics.getIndex(x, y);
		this.density[index] += amount;
	}

	public void addDensity(int x, int y, float amount, int size) {
		int halfSize = size / 2;
		for (int i = -halfSize; i < halfSize; i++) {
			for (int j = -halfSize; j < halfSize; j++) {
				int index = FluidDynamics.getIndex(x + i, y + j);
				this.density[index] += amount;
			}
		}
	}

	public void addVelocity(int x, int y, float xAmount, float yAmount) {
		int index = FluidDynamics.getIndex(x, y);
		this.xVelocity[index] += xAmount;
		this.yVelocity[index] += yAmount;
	}

	public void fade() {
		for (int i = 0; i < this.density.length; i++) {
			this.density[i] = FluidDynamics.constrain(this.density[i] - 0.1f, 0f, 255f);
			// FluidDynamics.friction(this.xVelocity, i, FluidDynamics.Friction);
			// FluidDynamics.friction(this.xPreviousVelocity, i, FluidDynamics.Friction);
			// FluidDynamics.friction(this.yVelocity, i, FluidDynamics.Friction);
			// FluidDynamics.friction(this.yPreviousVelocity, i, FluidDynamics.Friction);
		}
	}

	public void render(int[] pixels) {
		for (int i = 0; i < FluidDynamics.N; i++) {
			for (int j = 0; j < FluidDynamics.N; j++) {
				float d = this.density[FluidDynamics.getIndex(i, j)];
				int value = FluidDynamics.constrain((int) Math.floor(d), 0, 255);
				pixels[FluidDynamics.getIndex(i, j)] = (value << 16 | value << 8 | value);
			}
		}
		this.fade();
	}

	public void reset() {
		Arrays.fill(this.density, 0.0f);
		Arrays.fill(this.xVelocity, 0.0f);
		Arrays.fill(this.yVelocity, 0.0f);
		Arrays.fill(this.xPreviousVelocity, 0.0f);
		Arrays.fill(this.yPreviousVelocity, 0.0f);
	}

	public void step() {
		float visc = this.viscosity;
		float diff = this.diffusion;
		float dt = this.deltaTime;

		FluidDynamics.diffuse(1, this.xPreviousVelocity, this.xVelocity, visc, dt);
		FluidDynamics.diffuse(2, this.yPreviousVelocity, this.yVelocity, visc, dt);

		FluidDynamics.project(this.xPreviousVelocity, this.yPreviousVelocity, this.xVelocity, this.yVelocity);

		FluidDynamics.advect(1, this.xVelocity, this.xPreviousVelocity, this.xPreviousVelocity, this.yPreviousVelocity, dt);
		FluidDynamics.advect(2, this.yVelocity, this.yPreviousVelocity, this.xPreviousVelocity, this.yPreviousVelocity, dt);

		FluidDynamics.project(this.xVelocity, this.yVelocity, this.xPreviousVelocity, this.yPreviousVelocity);

		FluidDynamics.diffuse(0, this.size, this.density, diff, dt);
		FluidDynamics.advect(0, this.density, this.size, this.xVelocity, this.yVelocity, dt);
	}

	public void updateDeltaTime(float deltaTime) {
		this.deltaTime += deltaTime;
	}
}

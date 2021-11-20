package common;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Insets;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.awt.image.BufferStrategy;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.util.Random;

import javax.swing.JFrame;
import javax.swing.WindowConstants;

public class UI extends Canvas implements Runnable, MouseListener, MouseMotionListener {
	private static final String title = "Fluid Dynamics";
	private static final float diffusionConstant = 0.1f;
	private static final float viscocityConstant = 0.0004f;
	private static final float densityConstant = 255.0f;
	private static final float deltaTimeConstant = 0.0000000001f;
	private static final float accelerationConstant = 128f * 4f * FluidDynamics.Scale;

	private boolean isRunning = false;
	private FluidDynamics fluids;
	private BufferedImage image;
	private int[] pixels;
	private Random random;
	private int prevX;
	private int prevY;
	private boolean[] isPressed = new boolean[MouseEvent.BUTTON3 + 1];

	public UI(JFrame frame) {
		this.fluids = new FluidDynamics(0.0f, UI.diffusionConstant, UI.viscocityConstant);
		this.image = new BufferedImage(FluidDynamics.N, FluidDynamics.N, BufferedImage.TYPE_INT_RGB);
		this.pixels = ((DataBufferInt) this.image.getRaster().getDataBuffer()).getData();
		this.random = new Random();

		this.addMouseListener(this);
		this.addMouseMotionListener(this);
	}

	// ---------------------------------------------------

	public static void main(String[] args) {
		JFrame frame = new JFrame(UI.title);

		UI ui = new UI(frame);
		frame.getContentPane().add(ui);
		frame.setLocationRelativeTo(null);
		frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
		frame.addWindowListener(ui.getWindowListener());
		frame.setResizable(false);
		frame.setAlwaysOnTop(true);
		frame.pack();

		Insets inset = frame.getInsets();
		int width = inset.left + inset.right + FluidDynamics.N * FluidDynamics.Scale;
		frame.setSize(new Dimension(width, width));

		frame.setVisible(true);

		ui.start();
	}

	// ----------------------------------------------------

	public WindowListener getWindowListener() {
		return new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				UI.this.stop();
			}
		};
	}

	@Override
	public void mouseClicked(MouseEvent e) {
		this.isPressed[e.getButton()] = true;
		if (this.isPressed[MouseEvent.BUTTON2]) {
			// Right button
			this.fluids.reset();
		}
	}

	@Override
	public void mouseDragged(MouseEvent e) {
		if (this.isPressed[MouseEvent.BUTTON1]) {
			int ox = e.getX();
			int oy = e.getY();
			int x = ox / FluidDynamics.Scale;
			int y = oy / FluidDynamics.Scale;
			this.fluids.addDensity(x, y, UI.densityConstant, 8);
		}
		if (this.isPressed[MouseEvent.BUTTON3]) {
			int ox = e.getX();
			int oy = e.getY();
			int x = ox / FluidDynamics.Scale;
			int y = oy / FluidDynamics.Scale;
			if (Math.abs(ox - this.prevX) > 5 || Math.abs(oy - this.prevY) > 5) {
				float accelX = (ox - this.prevX) * UI.accelerationConstant;
				float accelY = (oy - this.prevY) * UI.accelerationConstant;
				this.fluids.addVelocity(x, y, accelX, accelY);
			}
		}
	}

	@Override
	public void mouseEntered(MouseEvent e) {}

	@Override
	public void mouseExited(MouseEvent e) {}

	@Override
	public void mouseMoved(MouseEvent e) {}

	@Override
	public void mousePressed(MouseEvent e) {
		this.isPressed[e.getButton()] = true;
		if (this.isPressed[MouseEvent.BUTTON3]) {
			this.prevX = e.getX();
			this.prevY = e.getY();
		}
	}

	@Override
	public void mouseReleased(MouseEvent e) {
		this.isPressed[e.getButton()] = false;
		if (this.isPressed[MouseEvent.BUTTON3]) {
			this.prevX = e.getX();
			this.prevY = e.getY();
		}
	}

	public void render() {
		BufferStrategy bs = this.getBufferStrategy();
		if (bs == null) {
			this.createBufferStrategy(2);
			bs = this.getBufferStrategy();
		}
		Graphics g = bs.getDrawGraphics();
		g.setColor(Color.black);
		g.fillRect(0, 0, this.getWidth(), this.getHeight());

		this.fluids.render(this.pixels);
		g.drawImage(this.image, 0, 0, FluidDynamics.N * FluidDynamics.Scale, FluidDynamics.N * FluidDynamics.Scale, null);

		g.dispose();
		bs.show();
	}

	@Override
	public void run() {
		final long nanosecondsPerSecond = 1000 * 1000 * 1000;
		final int targetFps = 60;
		final int targetUpdateTick = 60;
		final long updateInterval = nanosecondsPerSecond / targetUpdateTick;
		final long renderInterval = nanosecondsPerSecond / targetFps;

		long nextUpdate = System.nanoTime() + updateInterval;
		long nextRender = System.nanoTime() + renderInterval;
		long now = 0L;

		while (this.isRunning) {
			now = System.nanoTime();

			while (now > nextUpdate) {
				this.tick(now / nanosecondsPerSecond);
				nextUpdate = now + updateInterval;
			}

			while (now > nextRender) {
				try {
					this.render();
					Thread.sleep(1);
				}
				catch (InterruptedException e) {
					e.printStackTrace();
				}
				nextRender = now + renderInterval;
			}
		}
	}

	public void start() {
		this.isRunning = true;
		Thread thread = new Thread(this);
		thread.setName(UI.title);
		thread.start();
	}

	public void stop() {
		this.isRunning = false;
	}

	public void tick(float deltaTime) {
		this.fluids.updateDeltaTime(UI.deltaTimeConstant);
		this.fluids.step();
	}
}

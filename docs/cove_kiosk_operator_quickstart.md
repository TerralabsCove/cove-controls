# COVE Kiosk Operator Quickstart

## Purpose

This is the shortest path to start the kiosk, open the website, and verify that the robot moves in RViz.

## Start The System

From the repository root:

```bash
scripts/ubuntu/run_cove_kiosk.sh rviz-sim --build
```

If port `8080` is already in use:

```bash
scripts/ubuntu/run_cove_kiosk.sh rviz-sim --build --port 8092
```

## Open The Kiosk

Default:

- `http://localhost:8080`

Alternate port example:

- `http://localhost:8092`

## What Should Open

You should see:

- the kiosk website in your browser
- an RViz window
- the robot model visible in the RViz 3D view

## Run A Test Order

1. Open the kiosk website.
2. Tap the order button.
3. Enter a name.
4. Submit the order.

## What You Should See

### In the website

- order moves through:
  - `received`
  - `pouring`
  - `moving`
  - `arrived`

### In RViz

- the robot model moves in the main 3D viewport
- motion follows the kiosk order sequence
- you do not need to drag the robot manually

Important:

- the website does not control RViz interactive markers
- the robot should move on its own after an order is submitted

## If Nothing Happens

Check these in order:

1. Make sure you opened the same port the launcher printed.
2. Make sure you submitted an order.
3. Look at the newest RViz window, not an older one.
4. Make sure the robot model is visible in RViz.
5. Make sure `RobotModel` and `MotionPlanning` are enabled in the RViz left panel.

## If The Website Loads But Motion Is Wrong

Most common cause:

- the browser is pointed at an old kiosk backend on a different port

Fix:

1. stop old kiosk sessions
2. restart with a clean port
3. open the matching URL

Example:

```bash
scripts/ubuntu/run_cove_kiosk.sh rviz-sim --build --port 8092
```

Then open:

- `http://localhost:8092`

## Stop The System

In the terminal where the launcher is running:

```bash
Ctrl-C
```

## Operator Checklist

- start `rviz-sim`
- open the matching kiosk URL
- confirm RViz is open
- submit one test order
- verify the robot model moves
- stop with `Ctrl-C` when done

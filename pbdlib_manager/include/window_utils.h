/*
 * window_utils.h
 *
 * Helper functions to manage windows taking multiple monitors
 * into account.
 *
 * Authors: Philip Abbet
 */

#pragma once

#include <GLFW/glfw3.h>
#include <algorithm>


//-----------------------------------------------------------------------------
// Returns the monitor on which the provided window is located.
//
// This functionality doesn't exists in GLFW.
//-----------------------------------------------------------------------------
GLFWmonitor* get_current_monitor(GLFWwindow* window)
{
	int win_x, win_y, win_width, win_height;
	glfwGetWindowPos(window, &win_x, &win_y);
	glfwGetWindowSize(window, &win_width, &win_height);

	int nb_monitors;
	GLFWmonitor** monitors;
	monitors = glfwGetMonitors(&nb_monitors);

	int best_overlap = 0;
	GLFWmonitor* best_monitor = 0;

	for (int i = 0; i < nb_monitors; ++i) {
		const GLFWvidmode* mode = glfwGetVideoMode(monitors[i]);

		int monitor_x, monitor_y;
		glfwGetMonitorPos(monitors[i], &monitor_x, &monitor_y);

		int monitor_width = mode->width;
		int monitor_height = mode->height;

		int overlap =
			std::max(0, std::min(win_x + win_width, monitor_x + monitor_width) - std::max(win_x, monitor_x)) *
			std::max(0, std::min(win_y + win_height, monitor_y + monitor_height) - std::max(win_y, monitor_y)
		);

		if (best_overlap < overlap) {
			best_overlap = overlap;
			best_monitor = monitors[i];
		}
	}

	return best_monitor;
}


//-----------------------------------------------------------------------------
// Create a window as large as possiblem but adapted to the current monitor.
// On input, the provided dimensions are used as an hint for the desired aspect
// ratio. On output, the real dimensions of the window is returned.
//-----------------------------------------------------------------------------
GLFWwindow* create_window_at_optimal_size(const char* title,
										  int &width, int &height) {

	// First create a window of the desired size
	GLFWwindow* window = glfwCreateWindow(
		width, height, title, NULL, NULL
	);

	// Next determine on which monitor the window is located
	GLFWmonitor* monitor = get_current_monitor(window);

	// Then compute the "optimal" size of the window on that monitor
	const GLFWvidmode* mode = glfwGetVideoMode(monitor);

	int original_width = width;
	int original_height = height;

	if (mode->height >= 2000)
		height = 1800;
	else if (mode->height >= 1440)
		height = 1200;
	else if (mode->height >= 1200)
		height = 1000;
	else if (mode->height >= 1000)
		height = 900;
	else if (mode->height >= 900)
		height = 800;
	else
		height = 600;

	width = original_width * height / original_height;

	if (width >= mode->width)
	{
		width = mode->width - 100;
		height = original_height * width / original_width;
	}

	// Finally, resize the window and center it
	glfwSetWindowSize(window, width, height);
	glfwSetWindowPos(window, (mode->width - width) / 2, (mode->height - height) / 2);

	return window;
}

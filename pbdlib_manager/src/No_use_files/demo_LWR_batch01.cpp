/*
 * demo_LWR_batch01.cpp
 *
 * Locally weighted regression (LWR) with radial basis functions (RBF), using batch
 * computation
 *
 * Authors: Sylvain Calinon, Philip Abbet
 */


#include <stdio.h>
#include <armadillo>

#include <gfx2.h>
#include <gfx_ui.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw_gl2.h>
#include <imgui_internal.h>
#include <window_utils.h>

using namespace arma;


/***************************** ALGORITHM SECTION *****************************/

typedef std::vector<vec> vector_list_t;
typedef std::vector<mat> matrix_list_t;


//-----------------------------------------------------------------------------
// Contains all the parameters used by the algorithm. Some of them are
// modifiable through the UI, others are hard-coded.
//-----------------------------------------------------------------------------
struct parameters_t {
	int nb_RBF;   // Number of radial basis functions
	int nb_data;  // Number of datapoints in a trajectory
};


//-----------------------------------------------------------------------------
// Likelihood of datapoint(s) to be generated by a Gaussian parameterized by
// center and covariance.
//-----------------------------------------------------------------------------
arma::vec gaussPDF(vec Data, double Mu, double Sigma) {

	int nb_data = Data.n_rows;
	Data = Data - repmat(mat({ Mu }), nb_data, 1);

	vec prob = sum((Data / Sigma) % Data, 1);

	prob = exp(-0.5 * prob) / sqrt(2 * datum::pi * Sigma + DBL_MIN);

	return prob;
}


//-----------------------------------------------------------------------------
// Locally weighted regression (LWR) with radial basis functions (RBF)
//-----------------------------------------------------------------------------
std::tuple<mat, mat> compute_LWR(const parameters_t& parameters, const mat& demonstration) {

	// Set centroids equally spread in time
	vec mu_RBF = linspace<vec>(0, parameters.nb_data - 1, parameters.nb_RBF);

	// Set constant homogeneous covariance
	double sigma_RBF = 100.0;

	mat H = zeros(parameters.nb_RBF, parameters.nb_data);
	for (int i = 0; i < parameters.nb_RBF; ++i) {
		H(i, span::all) = gaussPDF(linspace<vec>(0, parameters.nb_data - 1, parameters.nb_data),
								   mu_RBF(i), sigma_RBF).t();
	}

	// Batch estimate (Least squares estimate of weights)
	mat w = solve(H.t(), demonstration.t());

	// Reconstruction of signal by weighted sum of radial basis functions
	mat reproduction = w.t() * H;

	return std::make_tuple(reproduction, H);
}


/****************************** HELPER FUNCTIONS *****************************/

static void error_callback(int error, const char* description) {
	fprintf(stderr, "Error %d: %s\n", error, description);
}


//-----------------------------------------------------------------------------
// Contains all the informations about a viewport
//-----------------------------------------------------------------------------
struct viewport_t {
	int x;
	int y;
	int width;
	int height;

	// Projection matrix parameters
	arma::vec projection_top_left;
	arma::vec projection_bottom_right;
	double projection_near;
	double projection_far;
};


//-----------------------------------------------------------------------------
// Helper function to setup a viewport
//-----------------------------------------------------------------------------
void setup_viewport(viewport_t* viewport, int x, int y, int width, int height,
					double near = -1.0, double far = 1.0) {

	viewport->x = x;
	viewport->y = y;
	viewport->width = width;
	viewport->height = height;
	viewport->projection_top_left = vec({ (double) -width / 2,
										  (double) height / 2 });
	viewport->projection_bottom_right = vec({ (double) width / 2,
											  (double) -height / 2 });
	viewport->projection_near = near;
	viewport->projection_far = far;
}


//-----------------------------------------------------------------------------
// Converts some coordinates from UI-space to OpenGL-space, taking the
// coordinates of a viewport into account
//-----------------------------------------------------------------------------
arma::vec ui2fb(const arma::vec& coords, const gfx2::window_size_t& window_size,
				const viewport_t& viewport) {
	arma::vec result = coords;

	// ui -> viewport
	result(0) = coords(0) * (float) window_size.fb_width / (float) window_size.win_width - viewport.x;
	result(1) = (window_size.win_height - coords(1)) *
				(float) window_size.fb_height / (float) window_size.win_height - viewport.y;

	// viewport -> fb
	result(0) = result(0) - (float) viewport.width * 0.5f;
	result(1) = result(1) - (float) viewport.height * 0.5f;

	return result;
}


//-----------------------------------------------------------------------------
// Converts some coordinates from OpenGL-space to UI-space, taking the
// coordinates of a viewport into account
//-----------------------------------------------------------------------------
arma::vec fb2ui(const arma::vec& coords, const gfx2::window_size_t& window_size,
				const viewport_t& viewport) {
	arma::vec result = coords;

	// fb -> viewport
	result(0) = coords(0) + (float) viewport.width * 0.5f;
	result(1) = coords(1) + (float) viewport.height * 0.5f;

	// viewport -> ui
	result(0) = (result(0) + viewport.x) * (float) window_size.win_width / (float) window_size.fb_width;

	result(1) = window_size.win_height - (result(1) + viewport.y) * (float) window_size.win_height / (float) window_size.fb_height;

	return result;
}


//-----------------------------------------------------------------------------
// Colors of the displayed lines and gaussians
//-----------------------------------------------------------------------------
const mat COLORS({
	{ 0.0,  0.0,  1.0  },
	{ 0.0,  0.5,  0.0  },
	{ 1.0,  0.0,  0.0  },
	{ 0.0,  0.75, 0.75 },
	{ 0.75, 0.0,  0.75 },
	{ 0.75, 0.75, 0.0  },
	{ 0.25, 0.25, 0.25 },
});


//-----------------------------------------------------------------------------
// Create a demonstration (with a length of 'parameters.nb_data') from a
// trajectory (of any length)
//-----------------------------------------------------------------------------
mat sample_trajectory(const vector_list_t& trajectory, const parameters_t& parameters) {

	// Resampling of the trajectory
	vec x(trajectory.size());
	vec y(trajectory.size());

	for (size_t i = 0; i < trajectory.size(); ++i) {
		x(i) = trajectory[i](0);
		y(i) = trajectory[i](1);
	}

	vec from_indices = linspace<vec>(0, trajectory.size() - 1, trajectory.size());
	vec to_indices = linspace<vec>(0, trajectory.size() - 1, parameters.nb_data);

	vec x2;
	vec y2;

	interp1(from_indices, x, to_indices, x2, "*linear");
	interp1(from_indices, y, to_indices, y2, "*linear");

	// Create the demonstration
	mat demo(2, x2.size());
	for (int i = 0; i < x2.size(); ++i) {
		demo(0, i) = x2[i];
		demo(1, i) = y2[i];
	}

	return demo;
}


//-----------------------------------------------------------------------------
// Contains all the needed infos about the state of the application (values of
// the parameters modifiable via the UI, which action the user is currently
// doing, ...)
//-----------------------------------------------------------------------------
struct gui_state_t {
	// Indicates if the user is currently drawing a new demonstration
	bool is_drawing_demonstration;

	// Indicates if the parameters dialog is displayed
	bool is_parameters_dialog_displayed;

	// Indicates if the parameters were modified through the UI
	bool are_parameters_modified;

	// Indicates if the reproductions must be recomputed
	bool must_recompute_LWR;

	// Parameters modifiable via the UI (they correspond to the ones declared
	// in parameters_t)
	int parameter_nb_RBF;
	int parameter_nb_data;
};


//-----------------------------------------------------------------------------
// Render the "demonstrations & model" viewport
//-----------------------------------------------------------------------------
void draw_demo_viewport(const viewport_t& viewport,
						const vector_list_t& current_trajectory,
						const mat& demonstration,
						const mat& reproduction) {

	glViewport(viewport.x, viewport.y, viewport.width, viewport.height);
	glScissor(viewport.x, viewport.y, viewport.width, viewport.height);
	glClearColor(0.7f, 0.7f, 0.7f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(viewport.projection_top_left(0), viewport.projection_bottom_right(0),
			viewport.projection_bottom_right(1), viewport.projection_top_left(1),
			viewport.projection_near, viewport.projection_far);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Draw the demonstration
	if (demonstration.n_cols > 0)
		gfx2::draw_line({0.0f, 0.0f, 0.0f}, demonstration);
	else if (current_trajectory.size() > 1)
		gfx2::draw_line(arma::fvec({0.33f, 0.97f, 0.33f}), current_trajectory);

	// Draw the reproduction
	if (reproduction.n_cols > 0) {
		glLineWidth(4.0f);
		gfx2::draw_line({1.0f, 0.0f, 0.0f}, reproduction);
		glLineWidth(1.0f);
	}
}


//-----------------------------------------------------------------------------
// Returns the dimensions that a plot should have inside the provided viewport
//-----------------------------------------------------------------------------
ivec get_plot_dimensions(const viewport_t& viewport) {

	const int MARGIN = 50;

	ivec result(2);
	result(0) = viewport.width - 2 * MARGIN;
	result(1) = viewport.height / 2 - 2 * MARGIN;

	return result;
}


//-----------------------------------------------------------------------------
// Render a "timeline" viewport
//-----------------------------------------------------------------------------
void draw_timeline_viewport(const gfx2::window_size_t& window_size,
							const viewport_t& viewport,
							const mat& demonstration, const mat& reproduction,
							const mat& H, int dimension) {

	glViewport(viewport.x, viewport.y, viewport.width, viewport.height);
	glScissor(viewport.x, viewport.y, viewport.width, viewport.height);
	glClearColor(0.9f, 0.9f, 0.9f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(viewport.projection_top_left(0), viewport.projection_bottom_right(0),
			viewport.projection_bottom_right(1), viewport.projection_top_left(1),
			viewport.projection_near, viewport.projection_far);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	ivec plot_dimensions = get_plot_dimensions(viewport);

	const int MARGIN = 50;
	const int TOP_OFFSET = 40;
	const int MARGIN_TIMELINE_TOP = MARGIN + TOP_OFFSET;
	const int MARGIN_TIMELINE_BOTTOM = MARGIN - TOP_OFFSET;


	//_____ Timeline plot __________

	ivec plot_top_left({ -plot_dimensions(0) / 2, viewport.height / 2 - MARGIN_TIMELINE_TOP });
	ivec plot_bottom_right({ plot_dimensions(0) / 2, MARGIN_TIMELINE_BOTTOM });

	// Axis labels
	ui::begin("Text");

	vec coords = fb2ui(vec({ -20.0, MARGIN_TIMELINE_BOTTOM - 5.0 }), window_size, viewport);
	ui::text(ImVec2(coords(0), coords(1)), "t", ImVec4(0,0,0,1));

	std::stringstream label;
	label << "x" << dimension;

	coords = fb2ui(vec({ double(-viewport.width / 2) + 10, viewport.height / 4 - TOP_OFFSET + 20.0 }),
				   window_size, viewport);
	ui::text(ImVec2(coords(0), coords(1)), label.str(), ImVec4(0,0,0,1));

	ui::end();

	// Draw the axes
	gfx2::draw_line(fvec({0.0f, 0.0f, 0.0f}),
					mat({ { double(plot_top_left(0)), double(plot_bottom_right(0)) },
						  { double(plot_bottom_right(1)), double(plot_bottom_right(1)) }
						})
	);

	gfx2::draw_line(fvec({0.0f, 0.0f, 0.0f}),
					mat({ { double(plot_top_left(0)), double(plot_top_left(0)) },
						  { double(plot_bottom_right(1)), double(plot_top_left(1)) }
						})
	);

	double scale_x = (double) plot_dimensions(0) / (demonstration.n_cols - 1);

	// Check if there is something to display
	if (demonstration.n_cols > 0) {

		double scale_y;

		if (dimension == 1)
			scale_y = (double) plot_dimensions(1) / viewport.width;
		else
			scale_y = (double) plot_dimensions(1) / viewport.height;

		// Draw the demonstration
		arma::mat datapoints(2, demonstration.n_cols);
		datapoints.row(0) = linspace<vec>(0, demonstration.n_cols - 1, demonstration.n_cols).t();
		datapoints.row(1) = demonstration.row(dimension - 1);

		datapoints(0, span::all) = datapoints(0, span::all) * scale_x - plot_dimensions(0) / 2;
		datapoints(1, span::all) *= scale_y;
		datapoints(1, span::all) += viewport.height / 4 - TOP_OFFSET;

		glLineWidth(2.0f);
		gfx2::draw_line({0.7f, 0.7f, 0.7f}, datapoints);
		glLineWidth(1.0f);

		glClear(GL_DEPTH_BUFFER_BIT);

		// Draw the reproduction
		datapoints.row(1) = reproduction.row(dimension - 1);
		datapoints(1, span::all) *= scale_y;
		datapoints(1, span::all) += viewport.height / 4 - TOP_OFFSET;

		glLineWidth(4.0f);
		gfx2::draw_line({1.0f, 0.0f, 0.0f}, datapoints);
		glLineWidth(1.0f);
	}


	//_____ RBFs plot __________

	plot_top_left = ivec({ -plot_dimensions(0) / 2, -MARGIN });
	plot_bottom_right = ivec({ plot_dimensions(0) / 2, -viewport.height / 2 + MARGIN });

	// Axis labels
	ui::begin("Text");

	coords = fb2ui(vec({ -20.0, -viewport.height / 2 + (MARGIN - 5.0) }), window_size, viewport);
	ui::text(ImVec2(coords(0), coords(1)), "t", ImVec4(0,0,0,1));

	coords = fb2ui(vec({ double(-viewport.width / 2) + 10, -viewport.height / 4 - 20.0 }),
				   window_size, viewport);
	ui::text(ImVec2(coords(0), coords(1)), "h", ImVec4(0,0,0,1));

	ui::end();

	// Draw the axes
	gfx2::draw_line(fvec({0.0f, 0.0f, 0.0f}),
					mat({ { double(plot_top_left(0)), double(plot_bottom_right(0)) },
						  { double(plot_bottom_right(1)), double(plot_bottom_right(1)) }
						})
	);

	gfx2::draw_line(fvec({0.0f, 0.0f, 0.0f}),
					mat({ { double(plot_top_left(0)), double(plot_top_left(0)) },
						  { double(plot_bottom_right(1)), double(plot_top_left(1)) }
						})
	);

	// Check if there is something to display
	if (demonstration.n_cols > 0) {

		double max_value = max(max(H));

		int color_index = 0;
		for (int i = 0; i < H.n_rows; ++i) {
			arma::mat datapoints(2, H.n_cols);
			datapoints.row(0) = linspace<vec>(0, H.n_cols - 1, H.n_cols).t();
			datapoints.row(1) = H.row(i) / max_value;

			datapoints(0, span::all) = datapoints(0, span::all) * scale_x - plot_dimensions(0) / 2;
			datapoints(1, span::all) *= (double) plot_dimensions(1);
			datapoints(1, span::all) -= viewport.height / 2 - MARGIN;

			arma::fvec color = arma::conv_to<arma::fvec>::from(COLORS.row(color_index));

			gfx2::draw_line(color, datapoints);

			glClear(GL_DEPTH_BUFFER_BIT);

			++color_index;
			if (color_index >= COLORS.n_rows)
				color_index = 0;
		}
	}
}


/******************************* MAIN FUNCTION *******************************/

int main(int argc, char **argv) {

	arma_rng::set_seed_random();

	// Parameters
	parameters_t parameters;
	parameters.nb_RBF  = 8;
	parameters.nb_data = 100;


	// Take 4k screens into account (framebuffer size != window size)
	gfx2::window_size_t window_size;
	window_size.win_width = 800;
	window_size.win_height = 800;
	window_size.fb_width = -1;	// Will be known later
	window_size.fb_height = -1;
	int viewport_width = 0;
	int viewport_height = 0;


	// Initialise GLFW
	glfwSetErrorCallback(error_callback);

	if (!glfwInit())
		return -1;

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// Open a window and create its OpenGL context
	GLFWwindow* window = create_window_at_optimal_size(
		"Demo Locally weighted regression (LWR)",
		window_size.win_width, window_size.win_height
	);

	glfwMakeContextCurrent(window);


	// Setup GLSL
	gfx2::init();
	glEnable(GL_SCISSOR_TEST);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_LINE_SMOOTH);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Setup ImGui
	ImGui::CreateContext();
	ImGui_ImplGlfwGL2_Init(window, true);


	// Viewports
	viewport_t viewport_demo;
	viewport_t viewport_timeline;


	// GUI state
	gui_state_t gui_state;
	gui_state.is_drawing_demonstration = false;
	gui_state.is_parameters_dialog_displayed = false;
	gui_state.are_parameters_modified = false;
	gui_state.must_recompute_LWR = false;
	gui_state.parameter_nb_RBF = parameters.nb_RBF;
	gui_state.parameter_nb_data = parameters.nb_data;


	// Main loop
	vector_list_t current_trajectory;
	mat demonstration;
	mat reproduction;
	mat H;
	int dimension = 1;

	while (!glfwWindowShouldClose(window)) {
		glfwPollEvents();

		// Detect when the window was resized
		if ((ImGui::GetIO().DisplaySize.x != window_size.win_width) ||
			(ImGui::GetIO().DisplaySize.y != window_size.win_height)) {

			window_size.win_width = ImGui::GetIO().DisplaySize.x;
			window_size.win_height = ImGui::GetIO().DisplaySize.y;

			glfwGetFramebufferSize(window, &window_size.fb_width, &window_size.fb_height);

			viewport_width = window_size.fb_width / 2 - 1;
			viewport_height = window_size.fb_height / 2 - 1;

			// Update all the viewports
			setup_viewport(&viewport_demo, 0, window_size.fb_height - viewport_height,
						   window_size.fb_width, viewport_height);

			setup_viewport(&viewport_timeline, 0, 0, window_size.fb_width, viewport_height);
		}


		// If the parameters changed, learn the model again
		if (gui_state.are_parameters_modified) {

			if (parameters.nb_data != gui_state.parameter_nb_data) {
				parameters.nb_data = gui_state.parameter_nb_data;
				demonstration = sample_trajectory(current_trajectory, parameters);
			}

			parameters.nb_RBF = gui_state.parameter_nb_RBF;

			gui_state.must_recompute_LWR = true;
			gui_state.are_parameters_modified = false;
		}


		// Recompute the LWR (if necessary)
		if (gui_state.must_recompute_LWR && (demonstration.n_cols > 0)) {
			std::tie(reproduction, H) = compute_LWR(parameters, demonstration);
			gui_state.must_recompute_LWR = false;
		}


		// Start the rendering
		ImGui_ImplGlfwGL2_NewFrame();

		glViewport(0, 0, window_size.fb_width, window_size.fb_height);
		glScissor(0, 0, window_size.fb_width, window_size.fb_height);
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		draw_demo_viewport(viewport_demo, current_trajectory,
						   demonstration, reproduction);

		draw_timeline_viewport(window_size, viewport_timeline,
							   demonstration, reproduction, H, dimension);


		// Window: Demonstration & reproduction
		ImGui::SetNextWindowSize(ImVec2(window_size.win_width, 36));
		ImGui::SetNextWindowPos(ImVec2(0, 0));
		ImGui::Begin("Demonstration & reproduction", NULL,
					 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings |
					 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
					 ImGuiWindowFlags_NoTitleBar
		);

		ImGui::Text("Demonstration & reproduction       ");
		ImGui::SameLine();

		if (ImGui::Button("Parameters"))
			gui_state.is_parameters_dialog_displayed = true;

		ImGui::End();


		// Window: Timeline
		ImGui::SetNextWindowSize(ImVec2(window_size.win_width, 36));
		ImGui::SetNextWindowPos(ImVec2(0, window_size.win_height / 2));
		ImGui::Begin("Timeline", NULL,
					 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings |
					 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
					 ImGuiWindowFlags_NoTitleBar
		);

		ImGui::Text("Timeline ");
		ImGui::SameLine();

		ImGui::RadioButton("1", &dimension, 1);
		ImGui::SameLine();
		ImGui::RadioButton("2", &dimension, 2);

		ImGui::End();


		// Window: Parameters
		ImGui::SetNextWindowSize(ImVec2(440, 106));
		ImGui::SetNextWindowPos(ImVec2((window_size.win_width - 440) / 2,
									   (window_size.win_height - 106) / 2));
		ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 255));

		if (gui_state.is_parameters_dialog_displayed)
			ImGui::OpenPopup("Parameters");

		if (ImGui::BeginPopupModal("Parameters", NULL,
								   ImGuiWindowFlags_NoResize |
								   ImGuiWindowFlags_NoSavedSettings)) {

			ImGui::SliderInt("Nb RBF", &gui_state.parameter_nb_RBF, 2, 20);
			ImGui::SliderInt("Nb data", &gui_state.parameter_nb_data, 100, 300);

			if (ImGui::Button("Close")) {
				ImGui::CloseCurrentPopup();
				gui_state.is_parameters_dialog_displayed = false;
				gui_state.are_parameters_modified = true;
			}

			ImGui::EndPopup();
		}

		ImGui::PopStyleColor();


		// GUI rendering
		ImGui::Render();
		ImGui_ImplGlfwGL2_RenderDrawData(ImGui::GetDrawData());

		// Swap buffers
		glfwSwapBuffers(window);

		// Keyboard input
		if (ImGui::IsKeyPressed(GLFW_KEY_ESCAPE))
			break;


		if (!gui_state.is_drawing_demonstration && !gui_state.is_parameters_dialog_displayed) {
			// Left click: start a new demonstration (only if not on the UI and in the
			// demonstrations viewport)
			if (ImGui::IsMouseClicked(GLFW_MOUSE_BUTTON_1)) {
				double mouse_x, mouse_y;
				glfwGetCursorPos(window, &mouse_x, &mouse_y);

				if ((mouse_y > 36) && (mouse_y <= window_size.win_height / 2))
				{
					gui_state.is_drawing_demonstration = true;

					current_trajectory.clear();
					demonstration.clear();
					reproduction.clear();

					vec coords = ui2fb({ mouse_x, mouse_y }, window_size, viewport_demo);
					current_trajectory.push_back(coords);
				}
			}
		} else if (gui_state.is_drawing_demonstration) {
			double mouse_x, mouse_y;
			glfwGetCursorPos(window, &mouse_x, &mouse_y);

			vec coords = ui2fb({ mouse_x, mouse_y }, window_size, viewport_demo);

			vec last_point = current_trajectory[current_trajectory.size() - 1];
			vec diff = abs(coords - last_point);

			if ((diff(0) > 1e-6) && (diff(1) > 1e-6))
				current_trajectory.push_back(coords);

			// Left mouse button release: end the demonstration creation
			if (!ImGui::IsMouseDown(GLFW_MOUSE_BUTTON_1)) {
				gui_state.is_drawing_demonstration = false;

				if (current_trajectory.size() > 1) {
					demonstration = sample_trajectory(current_trajectory, parameters);
					gui_state.must_recompute_LWR = true;
				}
			}
		}
	}


	// Cleanup
	ImGui_ImplGlfwGL2_Shutdown();
	glfwTerminate();

	return 0;
}

/*
 * demo_MPC_velocity01.cpp
 *
 * Interactive MPC demo, with batch LQT and repulsive field test
 *
 * Fabien Crépon, Philip Abbet, Sylvain Calinon, 2017
 */

#include <stdio.h>
#include <armadillo>
#include <mpc_utils.h>

#include <gfx2.h>
#include <gfx_ui.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw_gl2.h>
#include <window_utils.h>

using namespace arma;


/***************************** ALGORITHM SECTION *****************************/

//-----------------------------------------------------------------------------
// Contains all the parameters used by the algorithm. Some of them are
// modifiable through the UI, others are hard-coded.
//-----------------------------------------------------------------------------
struct parameters_t {
	int    nb_targets;             // Number of gaussians
	int    nb_repulsive_gaussians; // Number of repulsive gaussians
	int    nb_dimensions;
	int    order;
	double global_scale;           // global scale, avoids numerical issues in batch method
	double end_weight;             // forces movement to a stop (see stepwiseReference function)
	float  maximum_displacement;   // maximum displacement, used to compute R diagonal
	float  stroke_duration;        // duration of a stroke
	float  covscale;               // hack, field covariance scaling factor
	int    field_derivative;       // derivative used for additional field
	bool   use_field;
	bool   stepwise;
	double dt;
};

//-----------------------------------------------

mat compute_LQR(const parameters_t& parameters, const mat& Mu, const cube& Sigma,
				const mat& Mu_repulsive, const cube& Sigma_repulsive) {

	const double duration = parameters.stroke_duration * parameters.nb_targets;
	const int n = (int) (duration / parameters.dt);
	const int cDim = parameters.nb_dimensions * parameters.order;

	// Integration with higher order Taylor series expansion
	mat A, B;
	makeIntegratorChain(&A, &B, parameters.order);
	discretizeSystem(&A, &B, A, B, parameters.dt);
	A = kron(A, eye(parameters.nb_dimensions, parameters.nb_dimensions));
	B = kron(B, eye(parameters.nb_dimensions, parameters.nb_dimensions));

	// Reference
	mat Q, MuQ;

	if (parameters.stepwise) {
		stepwiseReference(&MuQ, &Q, Mu, Sigma, n, parameters.order,
						  parameters.nb_dimensions, parameters.end_weight);
	} else {
		viaPointReference(&MuQ, &Q, Mu, Sigma, n, parameters.order,
						  parameters.nb_dimensions, parameters.end_weight);
	}

	MuQ *= parameters.global_scale;
	Q /= parameters.global_scale * parameters.global_scale;

	// r based on oscillatory movement displacement
	double r = SHM_r(parameters.maximum_displacement, parameters.stroke_duration,
					 parameters.order);

	mat R = kron(eye(n - 1, n - 1), eye(parameters.nb_dimensions, parameters.nb_dimensions) * r);

	////////////////////////////////////
	// Batch LQR

	// Sx and Su matrices for batch LQR
	mat Su = zeros(cDim * n, parameters.nb_dimensions * (n - 1));
	mat Sx = kron(ones(n, 1),
				  eye(parameters.nb_dimensions * parameters.order,
					  parameters.nb_dimensions * parameters.order)
	);
	mat M = B;
	for (int i = 1; i < n; i++) {
		Sx.rows(i * cDim, n * cDim - 1) = Sx.rows(i * cDim, n * cDim - 1) * A;
		Su.submat(i * cDim, 0, (i + 1) * cDim - 1, i * parameters.nb_dimensions - 1) = M;
		M = join_horiz(A * M.cols(0, parameters.nb_dimensions - 1), M);
	}

	arma::vec x0 = MuQ.col(0);

	// Flatten Mu's
	mat Xi_hat = reshape(MuQ, cDim * n, 1);

	mat SuInvSigmaQ = Su.t() * Q;

	mat Rq = SuInvSigmaQ * Su + R;
	mat rq = SuInvSigmaQ * (Xi_hat - Sx * x0);

	// Repulsive field
	if (parameters.use_field) {
		int deriv = std::min(parameters.field_derivative, parameters.order - 2);

		for (int i = 0; i < Sigma_repulsive.n_slices; ++i) {
			mat MuQ_repulsive = repmat(Mu_repulsive.col(i), 1, n);
			mat invSigma_repulsive = zeros(cDim, cDim);

			invSigma_repulsive(span(parameters.nb_dimensions * deriv, parameters.nb_dimensions * deriv + 1),
							   span(parameters.nb_dimensions * deriv, parameters.nb_dimensions * deriv + 1)) =
				inv(Sigma_repulsive.slice(i) * parameters.covscale * parameters.covscale);

			mat Q_repulsive = kron(eye(n, n), invSigma_repulsive);

			mat Xi_hat_repulsive = reshape(MuQ_repulsive, cDim * n, 1);
			mat SuInvSigmaQ_repulsive = Su.t() * Q_repulsive;

			Rq = Rq + SuInvSigmaQ_repulsive * Su;
			rq = rq + SuInvSigmaQ_repulsive * (Xi_hat_repulsive - Sx * x0);
		}
	}

	// Least squares solution
	vec u = pinv(Rq) * rq;;
	mat Y = reshape(Sx * x0 + Su * u, cDim, n);

	return Y.rows(0, parameters.nb_dimensions - 1) / parameters.global_scale;
}


/****************************** HELPER FUNCTIONS *****************************/

static void error_callback(int error, const char* description){
	fprintf(stderr, "Error %d: %s\n", error, description);
}

//-----------------------------------------------

std::tuple<vec, mat> trans2d_to_gauss(const ui::Trans2d& gaussian_transforms,
					  				  const gfx2::window_size_t& window_size) {

	vec mu = gfx2::ui2fb_centered(vec({ gaussian_transforms.pos.x, gaussian_transforms.pos.y }),
								  window_size);

	vec t_x({
		gaussian_transforms.x.x * window_size.scale_x(),
		gaussian_transforms.x.y * window_size.scale_y()
	});

	vec t_y({
		gaussian_transforms.y.x * window_size.scale_x(),
		gaussian_transforms.y.y * window_size.scale_y()
	});

	mat RG = {
		{ t_x(0), t_y(0) },
		{ -t_x(1), -t_y(1) }
	};

	mat sigma = RG * RG.t();

	return std::make_tuple(mu, sigma);
}

//-----------------------------------------------

void gauss_to_trans2d(const vec& mu, const mat& sigma,
					  const gfx2::window_size_t& window_size, ui::Trans2d &t2d) {

	vec ui_mu = gfx2::fb2ui_centered(mu, window_size);

	t2d.pos.x = ui_mu(0);
	t2d.pos.y = ui_mu(1);

	mat V;
	vec d;
	eig_sym(d, V, sigma);
	mat VD = V * diagmat(sqrt(d));

	t2d.x.x = VD.col(0)(0) / window_size.scale_x();
	t2d.x.y = VD.col(1)(0) / window_size.scale_y();
	t2d.y.x = VD.col(0)(1) / window_size.scale_x();
	t2d.y.y = VD.col(1)(1) / window_size.scale_y();
}

//-----------------------------------------------

arma::mat gradient_2d(arma::mat X, int order = 1) {
	mat df = diff(X, 1, 1);
	mat db = fliplr(-diff(fliplr(X), 1, 1));

	mat d = (df + db) / 2;
	d = join_horiz(zeros(2, 1), d);
	d.col(0) = df.col(0);
	d.col(d.n_cols - 1) = db.col(db.n_cols - 1);

	if(order > 1)
		return gradient_2d(d, order - 1);

	return d;
}

//-----------------------------------------------

void plot(float x, float y, float w, float h, const vec& v, const fvec& color) {
	float v_min = v.min();
	float v_max = v.max();

	mat points(2, v.n_rows);
	points(0, span::all) = linspace<vec>(0, w, v.n_rows).t() + x;
	points(1, span::all) = (v.t() - v_min) / (v_max - v_min) * h + y;

	gfx2::draw_line(color, points);
}


/******************************* MAIN FUNCTION *******************************/

int main(int argc, char **argv){

	arma_rng::set_seed_random();

	// Parameters
	parameters_t parameters;
	parameters.nb_targets             = 4;
	parameters.nb_repulsive_gaussians = 1;
	parameters.nb_dimensions          = 2;
	parameters.order                  = 3;
	parameters.global_scale           = 0.001;
	parameters.end_weight             = 1.;
	parameters.maximum_displacement   = 0.1;
	parameters.stroke_duration        = 0.3;
	parameters.covscale               = 1.0;
	parameters.field_derivative       = 1;
	parameters.use_field              = true;
	parameters.stepwise               = false;
	parameters.dt                     = 0.01;


	// Take 4k screens into account (framebuffer size != window size)
	gfx2::window_size_t window_size;
	window_size.win_width = 800;
	window_size.win_height = 800;
	window_size.fb_width = -1;	// Will be known later
	window_size.fb_height = -1;


	// Initialise GLFW
	glfwSetErrorCallback(error_callback);

	if (!glfwInit())
		return -1;

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// Open a window and create its OpenGL context
	GLFWwindow* window = create_window_at_optimal_size(
		"Demo Velocity MPC", window_size.win_width, window_size.win_height
	);

	glfwMakeContextCurrent(window);


	// Setup GLSL
	gfx2::init();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_LINE_SMOOTH);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Setup ImGui
	ImGui::CreateContext();
	ImGui_ImplGlfwGL2_Init(window, true);


	// Covariances
	mat Mu = zeros(2, parameters.nb_targets);
	cube Sigma = zeros(2, 2, parameters.nb_targets);

	mat Mu0 = zeros(2, parameters.nb_repulsive_gaussians);
	cube Sigma0 = zeros(2, 2, parameters.nb_repulsive_gaussians);

	// Gaussian widgets
	std::vector<ui::Trans2d> t2ds(parameters.nb_targets, ui::Trans2d());
	std::vector<ui::Trans2d> t2ds0(parameters.nb_repulsive_gaussians, ui::Trans2d());

	// Main loop
	bool must_recompute = true;
	mat trajectory;

	while (!glfwWindowShouldClose(window)) {
		glfwPollEvents();

		// Detect when the window was resized
		if ((ImGui::GetIO().DisplaySize.x != window_size.win_width) ||
			(ImGui::GetIO().DisplaySize.y != window_size.win_height)) {

			bool first = (window_size.win_width == -1) || (window_size.fb_width == -1);

			// Retrieve the new window size
			glfwGetWindowSize(window, &window_size.win_width, &window_size.win_height);

			// Retrieve the new framebuffer size
			glfwGetFramebufferSize(window, &window_size.fb_width, &window_size.fb_height);

			// At the very first frame: initialise the UI widgets of the gaussians
			if (first && (window_size.fb_width > 0)) {

				Mu = randu(2, parameters.nb_targets);
				Mu.row(0) = Mu.row(0) * (window_size.fb_width - 200) - (window_size.fb_width / 2 - 100);
				Mu.row(1) = Mu.row(1) * (window_size.fb_height - 200) - (window_size.fb_height / 2 - 100);

				randomCovariances(
					&Sigma, Mu,
					vec({ 100 * (float) window_size.fb_width / window_size.win_width,
						  100 * (float) window_size.fb_height / window_size.win_height
					}),
					true, 0.0, 0.2
				);

				Mu0 = randu(2, parameters.nb_repulsive_gaussians);
				Mu0.row(0) = Mu0.row(0) * (window_size.fb_width - 200) - (window_size.fb_width / 2 - 100);
				Mu0.row(1) = Mu0.row(1) * (window_size.fb_height - 200) - (window_size.fb_height / 2 - 100);

				randomCovariances(
					&Sigma0, Mu0,
					vec({ 150 * (float) window_size.fb_width / window_size.win_width,
						  150 * (float) window_size.fb_height / window_size.win_height
					}),
					true, 0.0, 0.2
				);

				for (int i = 0; i < parameters.nb_targets; ++i) {
					gauss_to_trans2d(Mu.col(i), Sigma.slice(i), window_size, t2ds[i]);

					fmat rotation = gfx2::rotate(fvec({ 0.0f, 0.0f, 1.0f }), randu() * 2 * datum::pi);

					fvec x = rotation * fvec({ t2ds[i].x.x, t2ds[i].x.y, 0.0f, 0.0f });
					t2ds[i].x.x = x(0);
					t2ds[i].x.y = x(1);

					fvec y = rotation * fvec({ t2ds[i].y.x, t2ds[i].y.y, 0.0f, 0.0f });
					t2ds[i].y.x = y(0);
					t2ds[i].y.y = y(1);
				}

				for (int i = 0; i < parameters.nb_repulsive_gaussians; ++i) {
					gauss_to_trans2d(Mu0.col(i), Sigma0.slice(i), window_size, t2ds0[i]);

					fmat rotation = gfx2::rotate(fvec({ 0.0f, 0.0f, 1.0f }), randu() * 2 * datum::pi);

					fvec x = rotation * fvec({ t2ds0[i].x.x, t2ds0[i].x.y, 0.0f, 0.0f });
					t2ds0[i].x.x = x(0);
					t2ds0[i].x.y = x(1);

					fvec y = rotation * fvec({ t2ds0[i].y.x, t2ds0[i].y.y, 0.0f, 0.0f });
					t2ds0[i].y.x = y(0);
					t2ds0[i].y.y = y(1);
				}
			}
		}


		// Recompute the LQR when needed
		if (must_recompute && !ImGui::IsMouseDown(GLFW_MOUSE_BUTTON_1)) {
			trajectory = compute_LQR(parameters, Mu, Sigma, Mu0, Sigma0);
			must_recompute = false;
		}


		// Start of rendering
		ImGui_ImplGlfwGL2_NewFrame();

		glViewport(0, 0, window_size.fb_width, window_size.fb_height);
		glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-window_size.fb_width / 2, window_size.fb_width / 2,
				-window_size.fb_height / 2, window_size.fb_height / 2, -1.0f, 1.0f);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		glPushMatrix();


		// Draw the gaussians
		for( int i = 0; i < parameters.nb_targets; i++ ) {
			glClear(GL_DEPTH_BUFFER_BIT);
			gfx2::draw_gaussian_background(fvec({ 0.0f, 0.5f, 1.0f }), Mu.col(i), Sigma.slice(i));
		}
		glClear(GL_DEPTH_BUFFER_BIT);

		// Draw the repulsive gaussians
		for( int i = 0; i < parameters.nb_repulsive_gaussians; i++ ) {
			glClear(GL_DEPTH_BUFFER_BIT);
			gfx2::draw_gaussian_background(fvec({ 1.0f, 0.5f, 0.0f }), Mu0.col(i), Sigma0.slice(i));
		}
		glClear(GL_DEPTH_BUFFER_BIT);

		// Draw the motor plan
		glColor3f(0.5, 0.5 ,0.5);
		gfx2::draw_line(fvec({ 0.5f, 0.5f, 0.5f }), Mu);

		// Draw the trajectory
		gfx2::draw_line(fvec({ 0.0f, 0.0f, 1.0f }), trajectory);

		// Plot derivatives magnitude
		int plot_width = 200 * window_size.fb_width / window_size.win_width;
		int plot_height = 100 * window_size.fb_height / window_size.win_height;
		int plot_x = -window_size.fb_width / 2;
		int plot_y = -window_size.fb_height / 2 + 25 * window_size.fb_height / window_size.win_height;

		mat dx = gradient_2d(trajectory, 1);
		vec speed = sqrt(sum(dx % dx, 0)).t();
		plot(plot_x, plot_y, plot_width, plot_height, speed, {1.0f, 0.0f, 0.0f});

		ui::begin("Text");
		ui::text(ImVec2(10, window_size.win_height - 25), "velocity magnitude",
				 ImVec4(1, 0, 0, 1));
		ui::end();

		dx = gradient_2d(trajectory, 2);
		speed = sqrt(sum(dx % dx, 0)).t();
		plot(plot_x + plot_width, plot_y, plot_width, plot_height, speed, {0.0f, 0.0f, 1.0f});

		ui::begin("Text");
		ui::text(ImVec2(200 + 10, window_size.win_height - 25), "acceleration magnitude",
				 ImVec4(0, 0, 1, 1));
		ui::end();

		dx = gradient_2d(trajectory, 3);
		speed = sqrt(sum(dx % dx, 0)).t();
		plot(plot_x + 2 * plot_width, plot_y, plot_width, plot_height, speed, {0.0f, 0.5f, 0.0f});

		ui::begin("Text");
		ui::text(ImVec2(400 + 10, window_size.win_height - 25), "jerk magnitude",
				 ImVec4(0, 0.5, 0, 1));
		ui::end();

		glPopMatrix();


		// Gaussians UI
		ui::begin("Gaussian");

		for (int i = 0; i < parameters.nb_targets; ++i) {
			t2ds[i] = ui::affineSimple(i, t2ds[i]);

			vec mu;
			mat sigma;
			std::tie(mu, sigma) = trans2d_to_gauss(t2ds[i], window_size);

			must_recompute = must_recompute ||
							 (norm(mu - Mu.col(i)) > 1e-6) ||
							 (norm(sigma - Sigma.slice(i)) > 1e-6);

			Mu.col(i) = mu;
			Sigma.slice(i) = sigma;
		}

		for (int i = 0; i < parameters.nb_repulsive_gaussians; ++i) {
			t2ds0[i] = ui::affineSimple(i + parameters.nb_targets, t2ds0[i]);

			vec mu;
			mat sigma;
			std::tie(mu, sigma) = trans2d_to_gauss(t2ds0[i], window_size);

			must_recompute = must_recompute ||
							 (norm(mu - Mu0.col(i)) > 1e-6) ||
							 (norm(sigma - Sigma0.slice(i)) > 1e-6);

			Mu0.col(i) = mu;
			Sigma0.slice(i) = sigma;
		}

		ui::end();


		// Parameters window
		int winw = 400;
		ImGui::SetNextWindowPos(ImVec2(window_size.win_width - winw, 2));
		ImGui::Begin("Params", NULL, ImVec2(winw, 154), 0.5f,
					 ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
					 ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoSavedSettings
		);

		int previous_order = parameters.order;
		int previous_field_derivative = parameters.field_derivative;
		float previous_covscale = parameters.covscale;
		float previous_maximum_displacement = parameters.maximum_displacement;
		bool previous_use_field = parameters.use_field;
		bool previous_stepwise = parameters.stepwise;

		ImGui::SliderInt("Order", &parameters.order, 2, 7);
		ImGui::SliderInt("Field deriv", &parameters.field_derivative, 0, 4);
		ImGui::SliderFloat("covscale", &parameters.covscale, 0.1, 1.0);
		ImGui::SliderFloat("Max Displacement", &parameters.maximum_displacement, 0.1, 100.);
		ImGui::Checkbox("Use Field", &parameters.use_field);
		ImGui::Checkbox("Stepwise", &parameters.stepwise);

		ImGui::End();

		must_recompute = must_recompute ||
						 (parameters.order != previous_order) ||
						 (parameters.field_derivative != previous_field_derivative) ||
						 !gfx2::is_close(parameters.covscale, previous_covscale) ||
						 !gfx2::is_close(parameters.maximum_displacement, previous_maximum_displacement) ||
	 					 (parameters.use_field != previous_use_field) ||
						 (parameters.stepwise != previous_stepwise);


		// GUI rendering
		ImGui::Render();
		ImGui_ImplGlfwGL2_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);

		// Keyboard input
		if (ImGui::IsKeyPressed(GLFW_KEY_ESCAPE))
			break;
	}

	// Cleanup
	ImGui_ImplGlfwGL2_Shutdown();
	glfwTerminate();

	return 0;
}

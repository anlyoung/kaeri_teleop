/*
 * gfx2.h
 *
 * Rendering utility structures and functions based on OpenGL 2 (no shader)
 *
 * Authors: Philip Abbet
 */

#pragma once

#define ARMA_DONT_PRINT_ERRORS
#include <armadillo>
#include <map>

// Detect the platform
#ifdef _WIN32
	#define GFX_WINDOWS
	#include <windows.h>
#elif __APPLE__
	#define GFX_OSX
#elif __linux__ || __unix__ || defined(_POSIX_VERSION)
	#define GFX_LINUX
#else
	#error "Unknown platform"
#endif

// OpenGL includes
#ifdef GFX_WINDOWS
	// NOT TESTED HERE
	#include <windows.h>
	#include <GL/glu.h>
	#ifndef GL_CLAMP_TO_EDGE
		#define GL_CLAMP_TO_EDGE 0x812F
	#endif
#elif defined GFX_LINUX
	#ifndef __glew_h__
		#define GL_GLEXT_PROTOTYPES
		#include <GL/glew.h>
		#include <GL/glu.h>
		#include <GL/gl.h>
		#include <GL/glx.h>
	#endif
#elif defined GFX_OSX
	#ifndef __glew_h__
		#define GL_GLEXT_PROTOTYPES
		#include <GL/glew.h>
	#endif
	#include <OpenGL/glu.h>
	#include <OpenGL/OpenGL.h>
	#include <OpenGL/gl.h>
	#include <OpenGL/glext.h>
#endif



namespace gfx2
{
	/**************************** UTILITY FUNCTIONS **************************/

	//-------------------------------------------------------------------------
	// Holds the sizes of the window and of the OpenGL front-buffer (they might
	// be different, for instance on a 4K screen)
	//-------------------------------------------------------------------------
	struct window_size_t {
		int win_width;
		int win_height;
		int fb_width;
		int fb_height;

		inline float scale_x() const {
			return (float) fb_width / (float) win_width;
		}

		inline float scale_y() const {
			return (float) fb_height / (float) win_height;
		}
	};

	//-------------------------------------------------------------------------
	// Initialisations
	//-------------------------------------------------------------------------
	void init();

	//-------------------------------------------------------------------------
	// Convert radian to degrees
	//-------------------------------------------------------------------------
	double deg2rad(double deg);

	//-------------------------------------------------------------------------
	// Returns the sinus of an angle in degrees
	//-------------------------------------------------------------------------
	double sin_deg(double deg);

	//-------------------------------------------------------------------------
	// Returns the cosinus of an angle in degrees
	//-------------------------------------------------------------------------
	double cos_deg(double deg);

	//-------------------------------------------------------------------------
	// Indicates if two values are close enough
	//-------------------------------------------------------------------------
	bool is_close(float a, float b, float epsilon = 1e-6f);

	//-------------------------------------------------------------------------
	// Converts some coordinates from UI-space to OpenGL-space
	//
	// UI coordinates range from (0, 0) to (win_width, win_height)
	// OpenGL coordinates range from (0, fb_height) to (fb_width, 0)
	//-------------------------------------------------------------------------
	arma::vec ui2fb(const arma::vec& coords, const window_size_t& window_size);

	//-------------------------------------------------------------------------
	// Converts some coordinates from UI-space to OpenGL-space
	//
	// UI coordinates range from (0, 0) to (win_width, win_height)
	// OpenGL coordinates range from (0, fb_height) to (fb_width, 0)
	//-------------------------------------------------------------------------
	arma::vec ui2fb(const arma::vec& coords, int win_width, int win_height,
					int fb_width, int fb_height);

	//-------------------------------------------------------------------------
	// Converts some coordinates from UI-space to OpenGL-space
	//
	// UI coordinates range from (0, 0) to (win_width, win_height)
	// OpenGL coordinates range from (-fb_width / 2, fb_height / 2) to
	// (fb_width / 2, -fb_height / 2)
	//-------------------------------------------------------------------------
	arma::vec ui2fb_centered(const arma::vec& coords, const window_size_t& window_size);

	//-------------------------------------------------------------------------
	// Converts some coordinates from UI-space to OpenGL-space
	//
	// UI coordinates range from (0, 0) to (win_width, win_height)
	// OpenGL coordinates range from (-fb_width / 2, fb_height / 2) to
	// (fb_width / 2, -fb_height / 2)
	//-------------------------------------------------------------------------
	arma::vec ui2fb_centered(const arma::vec& coords, int win_width, int win_height,
							 int fb_width, int fb_height);

	//-------------------------------------------------------------------------
	// Converts some coordinates from OpenGL-space to UI-space
	//
	// OpenGL coordinates range from (-fb_width / 2, fb_height / 2) to
	// (fb_width / 2, -fb_height / 2)
	// UI coordinates range from (0, 0) to (win_width, win_height)
	//-------------------------------------------------------------------------
	arma::vec fb2ui(const arma::vec& coords, int win_width, int win_height,
					int fb_width, int fb_height);

	//-------------------------------------------------------------------------
	// Converts some coordinates from OpenGL-space to UI-space
	//
	// OpenGL coordinates range from (0, fb_height) to (fb_width, 0)
	// UI coordinates range from (0, 0) to (win_width, win_height)
	//-------------------------------------------------------------------------
	arma::vec fb2ui(const arma::vec& coords, const window_size_t& window_size);

	//-------------------------------------------------------------------------
	// Converts some coordinates from OpenGL-space to UI-space
	//
	// OpenGL coordinates range from (0, fb_height) to (fb_width, 0)
	// UI coordinates range from (0, 0) to (win_width, win_height)
	//-------------------------------------------------------------------------
	arma::vec fb2ui_centered(const arma::vec& coords, int win_width, int win_height,
							 int fb_width, int fb_height);

	//-------------------------------------------------------------------------
	// Converts some coordinates from OpenGL-space to UI-space
	//
	// OpenGL coordinates range from (-fb_width / 2, fb_height / 2) to
	// (fb_width / 2, -fb_height / 2)
	// UI coordinates range from (0, 0) to (win_width, win_height)
	//-------------------------------------------------------------------------
	arma::vec fb2ui_centered(const arma::vec& coords, const window_size_t& window_size);


	/*********************** PROJECTION & VIEW MATRICES **********************/

	//-------------------------------------------------------------------------
	// Returns a perspective projection matrix
	//-------------------------------------------------------------------------
	arma::fmat perspective(float fovy, float aspect, float zNear, float zFar);

	// //-------------------------------------------------------------------------
	// // Returns a orthographic projection matrix
	// //-------------------------------------------------------------------------
	// arma::fmat orthographic(float width, float height, float zNear, float zFar);

	//-------------------------------------------------------------------------
	// Returns a view matrix
	//-------------------------------------------------------------------------
	arma::fmat lookAt(const arma::fvec& position, const arma::fvec& target,
					  const arma::fvec& up);


	/**************************** TRANSFORMATIONS ****************************/

	//-------------------------------------------------------------------------
	// Holds all the transformations needed for a 3D entity
	//
	// Can be organised in a hierarchy, where the parent transforms affect the
	// children ones
	//-------------------------------------------------------------------------
	struct transforms_t {

		// Constructor
		transforms_t()
		: parent(0)
		{
			position.zeros(3);
			rotation.eye(4, 4);
		}

		transforms_t* parent;

		arma::fvec position;
		arma::fmat rotation;
	};

	//-------------------------------------------------------------------------
	// Returns a rotation matrix given an axis and an angle (in radian)
	//-------------------------------------------------------------------------
	arma::fmat rotate(const arma::fvec& axis, float angle);

	//-------------------------------------------------------------------------
	// Returns the rotation matrix to go from one direction to another one
	//-------------------------------------------------------------------------
	arma::fmat rotation(const arma::fvec& from, const arma::fvec& to);

	//-------------------------------------------------------------------------
	// Compute the translation and rotation to apply to a list of 3D points A to
	// obtain the list of 3D points B.
	//
	// Points are organised in columns:
	//	[ x0 x1 x2 ...
	//	  y0 y1 y2
	//	  z0 z1 z2 ]
	//-------------------------------------------------------------------------
	void rigid_transform_3D(const arma::fmat& A, const arma::fmat& B,
							arma::fmat &rotation, arma::fvec &translation);

	//-------------------------------------------------------------------------
	// Returns the full world transformation matrix corresponding to the given
	// transforms structure, taking all its parent hierarchy into account
	//-------------------------------------------------------------------------
	arma::fmat worldTransforms(const transforms_t* transforms);

	//-------------------------------------------------------------------------
	// Returns the full world position corresponding to the given transforms
	// structure, taking all its parent hierarchy into account
	//-------------------------------------------------------------------------
	arma::fvec worldPosition(const transforms_t* transforms);

	//-------------------------------------------------------------------------
	// Returns the full world rotation matrix corresponding to the given
	// transforms structure, taking all its parent hierarchy into account
	//-------------------------------------------------------------------------
	arma::fmat worldRotation(const transforms_t* transforms);


	/******************************* LIGHTNING *******************************/

	//-------------------------------------------------------------------------
	// Holds all the needed informations about a point light
	//-------------------------------------------------------------------------
	struct point_light_t {
		transforms_t	transforms;
		arma::fvec		color;
	};

	//-------------------------------------------------------------------------
	// A list of lights
	//-------------------------------------------------------------------------
	typedef std::vector<point_light_t> light_list_t;


	/******************************** TEXTURES *******************************/

	//-------------------------------------------------------------------------
	// Holds all the needed informations about a texture
	//-------------------------------------------------------------------------
	struct texture_t {
		GLuint id;
		GLuint width;
		GLuint height;
		GLenum format;
		GLenum type;

		union {
			float* pixels_f;
			unsigned char* pixels_b;
		};
	};

	//-------------------------------------------------------------------------
	// Create a texture
	//-------------------------------------------------------------------------
	texture_t create_texture(int width, int height, GLenum format, GLenum type);

	//-------------------------------------------------------------------------
	// Create a texture
	//-------------------------------------------------------------------------
	void destroy(texture_t &texture);


	/********************************* MESHES ********************************/

	//-------------------------------------------------------------------------
	// Holds all the needed informations about a mesh
	//-------------------------------------------------------------------------
	struct model_t {
		GLenum			mode;

		// Vertex data
		GLuint			nb_vertices;
		GLfloat*		vertex_buffer;
		GLfloat*		normal_buffer;
		GLfloat*		uv_buffer;

		// Transforms
		transforms_t	transforms;

		// Material
		arma::fvec		ambiant_color;
		arma::fvec		diffuse_color;
		arma::fvec		specular_color;
		float			specular_power;
		texture_t		texture;

		// Other
		bool			lightning_enabled;
		bool			use_one_minus_src_alpha_blending;
	};

	//-------------------------------------------------------------------------
	// Represent a list of models
	//-------------------------------------------------------------------------
	typedef std::vector<gfx2::model_t> model_list_t;

	//-------------------------------------------------------------------------
	// Create a rectangular mesh, colored (no lightning)
	//-------------------------------------------------------------------------
	model_t create_rectangle(const arma::fvec& color, float width, float height,
							 const arma::fvec& position = arma::zeros<arma::fvec>(3),
							 const arma::fmat& rotation = arma::eye<arma::fmat>(4, 4),
							 transforms_t* parent_transforms = 0);

	//-------------------------------------------------------------------------
	// Create a rectangular mesh, textured (no lightning)
	//-------------------------------------------------------------------------
	model_t create_rectangle(const texture_t& texture, float width, float height,
							 const arma::fvec& position = arma::zeros<arma::fvec>(3),
							 const arma::fmat& rotation = arma::eye<arma::fmat>(4, 4),
							 transforms_t* parent_transforms = 0);

	//-------------------------------------------------------------------------
	// Create a square mesh, colored (no lightning)
	//-------------------------------------------------------------------------
	model_t create_square(const arma::fvec& color, float size,
						  const arma::fvec& position = arma::zeros<arma::fvec>(3),
						  const arma::fmat& rotation = arma::eye<arma::fmat>(4, 4),
						  transforms_t* parent_transforms = 0);

	//-------------------------------------------------------------------------
	// Create a sphere mesh, lighted
	//-------------------------------------------------------------------------
	model_t create_sphere(float radius = 1.0f,
						  const arma::fvec& position = arma::zeros<arma::fvec>(3),
						  const arma::fmat& rotation = arma::eye<arma::fmat>(4, 4),
						  transforms_t* parent_transforms = 0);

	//-------------------------------------------------------------------------
	// Create a line mesh, colored (no lightning), from a matrix containing the
	// point coordinates
	//-------------------------------------------------------------------------
	model_t create_line(const arma::fvec& color, const arma::mat& points,
						const arma::fvec& position = arma::zeros<arma::fvec>(3),
						const arma::fmat& rotation = arma::eye<arma::fmat>(4, 4),
						transforms_t* parent_transforms = 0,
						bool line_strip = true);

	//-------------------------------------------------------------------------
	// Create a line mesh, colored (no lightning), from an array containing the
	// point coordinates
	//-------------------------------------------------------------------------
	model_t create_line(const arma::fvec& color, const std::vector<arma::vec>& points,
						const arma::fvec& position = arma::zeros<arma::fvec>(3),
						const arma::fmat& rotation = arma::eye<arma::fmat>(4, 4),
						transforms_t* parent_transforms = 0,
						bool line_strip = true);

	//-------------------------------------------------------------------------
	// Create a mesh, colored (no lightning), from a matrix containing the
	// vertex coordinates
	//-------------------------------------------------------------------------
	model_t create_mesh(const arma::fvec& color, const arma::mat& vertices,
						const arma::fvec& position = arma::zeros<arma::fvec>(3),
						const arma::fmat& rotation = arma::eye<arma::fmat>(4, 4),
						transforms_t* parent_transforms = 0);

	//-------------------------------------------------------------------------
	// Create a mesh representing a gaussian, colored (no lightning)
	//-------------------------------------------------------------------------
	model_t create_gaussian_background(const arma::fvec& color, const arma::vec& mu,
									   const arma::mat& sigma,
									   const arma::fvec& position = arma::zeros<arma::fvec>(3),
									   const arma::fmat& rotation = arma::eye<arma::fmat>(4, 4),
									   transforms_t* parent_transforms = 0);

	//-------------------------------------------------------------------------
	// Create a line mesh representing a gaussian border, colored (no lightning)
	//-------------------------------------------------------------------------
	model_t create_gaussian_border(const arma::fvec& color, const arma::vec& mu,
								   const arma::mat& sigma,
								   const arma::fvec& position = arma::zeros<arma::fvec>(3),
								   const arma::fmat& rotation = arma::eye<arma::fmat>(4, 4),
								   transforms_t* parent_transforms = 0);

	//-------------------------------------------------------------------------
	// Release the OpenGL resources used by the model
	//-------------------------------------------------------------------------
	void destroy(model_t &model);


	/******************************* RENDERING *******************************/

	//-------------------------------------------------------------------------
	// Render a mesh
	//-------------------------------------------------------------------------
	bool draw(const model_t& model, const light_list_t& lights);

	//-------------------------------------------------------------------------
	// Render a mesh (shortcut when lights aren't used)
	//-------------------------------------------------------------------------
	inline bool draw(const model_t& model)
	{
		light_list_t lights;
		return draw(model, lights);
	}

	//-------------------------------------------------------------------------
	// Render a rectangular mesh, colored (no lightning)
	//-------------------------------------------------------------------------
	bool draw_rectangle(const arma::fvec& color, float width, float height,
						const arma::fvec& position = arma::zeros<arma::fvec>(3),
						const arma::fmat& rotation = arma::eye<arma::fmat>(4, 4));

	//-------------------------------------------------------------------------
	// Render a rectangular mesh, textured (no lightning)
	//-------------------------------------------------------------------------
	bool draw_rectangle(const texture_t& texture, float width, float height,
						const arma::fvec& position = arma::zeros<arma::fvec>(3),
						const arma::fmat& rotation = arma::eye<arma::fmat>(4, 4));

	//-------------------------------------------------------------------------
	// Render a line, colored (no lightning), from a matrix containing the
	// point coordinates
	//-------------------------------------------------------------------------
	bool draw_line(const arma::fvec& color, const arma::mat& points,
				   const arma::fvec& position = arma::zeros<arma::fvec>(3),
				   const arma::fmat& rotation = arma::eye<arma::fmat>(4, 4));

	//-------------------------------------------------------------------------
	// Render a line, colored (no lightning), from an array containing the
	// point coordinates
	//-------------------------------------------------------------------------
	bool draw_line(const arma::fvec& color, const std::vector<arma::vec>& points,
				   const arma::fvec& position = arma::zeros<arma::fvec>(3),
				   const arma::fmat& rotation = arma::eye<arma::fmat>(4, 4));

	//-------------------------------------------------------------------------
	// Render a mesh, colored (no lightning), from a matrix containing the
	// vertex coordinates
	//-------------------------------------------------------------------------
	bool draw_mesh(const arma::fvec& color, const arma::mat& vertices,
				   const arma::fvec& position = arma::zeros<arma::fvec>(3),
				   const arma::fmat& rotation = arma::eye<arma::fmat>(4, 4));

	//-------------------------------------------------------------------------
	// Render a gaussian, colored (no lightning)
	//-------------------------------------------------------------------------
	bool draw_gaussian(const arma::fvec& color, const arma::vec& mu,
					   const arma::mat& sigma, bool background = true,
					   bool border = true);

	//-------------------------------------------------------------------------
	// Render the border of a gaussian, colored (no lightning)
	//-------------------------------------------------------------------------
	inline bool draw_gaussian_border(const arma::fvec& color, const arma::vec& mu,
									 const arma::mat& sigma) {

		return draw_gaussian(color, mu, sigma, false, true);
	}

	//-------------------------------------------------------------------------
	// Render the background of a gaussian, colored (no lightning)
	//-------------------------------------------------------------------------
	inline bool draw_gaussian_background(const arma::fvec& color, const arma::vec& mu,
										 const arma::mat& sigma) {

		return draw_gaussian(color, mu, sigma, true, false);
	}


	/****************************** RAY CASTING ******************************/

	//-------------------------------------------------------------------------
	// Represents a 3D ray (in world coordinates)
	//-------------------------------------------------------------------------
	struct ray_t {
		arma::fvec origin;
		arma::fvec direction;
	};

	ray_t create_ray(const arma::fvec& origin, int mouse_x, int mouse_y,
					 const arma::fmat& view, const arma::fmat& projection,
					 int window_width, int window_height);

	bool intersects(const ray_t& ray, const arma::fvec& center, float radius,
					arma::fvec &result);


	/******************************** OTHERS *********************************/

	//-------------------------------------------------------------------------
	// Returns the vertices needed to create a mesh representing the background
	// a gaussian
	//
	// The result is a matrix of shape (2, nb_points * 3)
	//-------------------------------------------------------------------------
	arma::mat get_gaussian_background_vertices(const arma::vec& mu, const arma::mat& sigma,
											   int nb_points = 60);

	//-------------------------------------------------------------------------
	// Returns the vertices needed to create a line representing the border of
	// a gaussian
	//
	// If line_strip is true:
	//  - The result is a matrix of shape (2, nb_points)
	//  - The rendering mode must be GL_LINE_STRIP
	//
	// If line_strip is false:
	//  - The result is a matrix of shape (2, nb_points * 2)
	//  - The rendering mode must be GL_LINES
	//-------------------------------------------------------------------------
	arma::mat get_gaussian_border_vertices(const arma::vec& mu, const arma::mat& sigma,
										   int nb_points = 60, bool line_strip = true);

};

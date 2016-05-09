#ifndef PARAMETERS_SIMULATION_HPP_
#define PARAMETERS_SIMULATION_HPP_

struct Params_Simu {	

    struct global {
        static constexpr float discretization = 1e5;	// Precision expected for values given by Map-Elite (number of values between 0 and 1)
        static constexpr int intensityOfStab = 20;		// Intensity of the stabilization (bigger it is, slower is the stabilization)
    };

	// Table parameters
	struct table {
        static constexpr float dist_arm_x = 0.45;		// Distance between the arm and the center of the table (x axis)
        static constexpr float dist_arm_y = 0;			// Distance between the arm and the center of the table (y axis)
        static constexpr float dist_arm_z = 0;			// Distance between the arm and the center of the table (z axis)
        static constexpr float length = 1.6;			// Length of the table
        static constexpr float width = 0.8;				// Width of the table
        static constexpr float height = 0.2;			// Height of the table
	};
	// Cube parameters
	struct cube {
        static constexpr float dist_arm_x = 0.45;		// Distance between the arm and the cube (x axis)
        static constexpr float dist_arm_y = 0;			// Distance between the arm and the cube (y axis)
        static constexpr float dist_arm_z = 0;			// Distance between the arm and the cube (z axis)
        static constexpr float size = 0.052;			// Size of the cube
        static constexpr float mass = 0.094;			// Mass of the cube
        static constexpr float massCoeff = 1;			// Can be modified in the code by calling setMass_coeff (realMass = mass*massCoeff)
        static constexpr float rotation = 0;			// Rotation of the cube
        static constexpr float rotationCoeff = 0;		// Can be modified in the code by calling setRotation_coeff (realRotation = rotation+rotationCoeff)

        static constexpr float x_variance = .01;
        static constexpr float y_variance = .01;
        static constexpr float rot_variance = .1;
    };
    // Grid parameters (size (in meters) for physicals dimensions and position relative to the arm)
    struct grid {
        static constexpr float size_x = 0.65; //1.2;			// The size in meters of the x axis of the grid
        static constexpr float size_y = 0.8;			// The size in meters of the y axis of the grid
        static constexpr float size_z = 0.75; //1.0:			// The size in meters of the z axis of the grid
        static constexpr float gap_x = 0.05; //0.3;				// Position of the center of the grid to the cube (x axis)
        static constexpr float gap_y = 0.0;				// Position of the center of the grid to the cube (y axis)
        static constexpr float gap_z = 0.4; //0.5;				// Position of the center of the grid to the cube (z axis : from ground)
    };

};

class Data {

public:

    Data() {
        setBorder_left_x(Params_Simu::cube::dist_arm_x + Params_Simu::grid::gap_x - (Params_Simu::grid::size_x/2.0f));
        setBorder_left_y(Params_Simu::cube::dist_arm_y + Params_Simu::grid::gap_y - (Params_Simu::grid::size_y/2.0f));
        setBorder_left_z(Params_Simu::cube::dist_arm_z + Params_Simu::grid::gap_z - (Params_Simu::grid::size_z/2.0f));
        setBorder_right_x(Params_Simu::cube::dist_arm_x + Params_Simu::grid::gap_x + (Params_Simu::grid::size_x/2.0f));
        setBorder_right_y(Params_Simu::cube::dist_arm_y + Params_Simu::grid::gap_y + (Params_Simu::grid::size_y/2.0f));
        setBorder_right_z(Params_Simu::cube::dist_arm_z + Params_Simu::grid::gap_z + (Params_Simu::grid::size_z/2.0f));
		
        setMass_coeff(Params_Simu::cube::massCoeff);
        setRotation_coeff(Params_Simu::cube::rotationCoeff);
    }
	
    ~Data() {
    }
	
	
    /**
     * @brief get the real mass of the cube
     * @return the mass according to its ratio
     */
    float getRealCubeMass() {
        return Params_Simu::cube::mass * getMass_coeff();
    }
	
    /**
     * @brief get the real rotation of the cube
     * @return the rotation according to its ratio
     */
    float getRealCubeRotation() {
        if(getRotation_coeff() == 0)
            return 0;
        return Params_Simu::cube::rotation + M_PI/float(getRotation_coeff());
    }
	
	
    // Getters / Setters
    float getBorder_left_x() { return _border_left_x; }
    void setBorder_left_x(float border_left_x) { _border_left_x = border_left_x; }
    float getBorder_left_y() { return _border_left_y; }
    void setBorder_left_y(float border_left_y) { _border_left_y = border_left_y; }
    float getBorder_left_z() { return _border_left_z; }
    void setBorder_left_z(float border_left_z) { _border_left_z = border_left_z; }
    float getBorder_right_x() { return _border_right_x; }
    void setBorder_right_x(float border_right_x) { _border_right_x = border_right_x; }
    float getBorder_right_y() { return _border_right_y; }
    void setBorder_right_y(float border_right_y) { _border_right_y = border_right_y; }
    float getBorder_right_z() { return _border_right_z; }
    void setBorder_right_z(float border_right_z) { _border_right_z = border_right_z; }
	
    float getMass_coeff() { return _coefficient_cube; }
    void setMass_coeff(float coefficient_cube) { _coefficient_cube = coefficient_cube; }
    float getRotation_coeff() { return _rotation_coeff; }
    void setRotation_coeff(float rotation_coeff) { _rotation_coeff = rotation_coeff; }

private:

    float _border_left_x;
    float _border_left_y;
    float _border_left_z;
    float _border_right_x;
    float _border_right_y;
    float _border_right_z;
	
    float _coefficient_cube;
    float _rotation_coeff;
};


#endif

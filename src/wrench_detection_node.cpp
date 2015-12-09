// File name: wrench_detection_node.cpp
// Project specifics: Experimental Robotics Fall 2015 Final Project
// Author: Richard Wayne Fedora Jr. 
// Email: rfedora1@vt.edu
// Phone: (706) 254-8887
// Date of most recent edit: Friday, December 4, 2015
// 
// Code synopsis: implementation of RGB-D blending, background subtraction (depth segmentation), edge detection, Hough Circle Detector, and data fusion to recognize and localize a wrench and valve stem

# include <ros/ros.h>
# include <std_msgs/Float64MultiArray.h>
# include <armadillo>
# include <cmath>

void imCallback(const std_msgs::Float64MultiArray::ConstPtr& im_msg);
ros::Publisher mat_pub;
ros::Publisher mat_pub2;

arma::mat im;
arma::cube H;
int radius_max = 50;
int radius_min = 10;
int theta = 360;
int main(int argc, char **argv){

    ros::init(argc, argv, "wrench_detection_node");
    ros::NodeHandle n;
    // Declare publisher after initializing ros and nodehandle
    mat_pub = n.advertise<std_msgs::Float64MultiArray>("center_indices_by_location", 1000);
    mat_pub2 = n.advertise<std_msgs::Float64MultiArray>("center_indices_by_votes", 1000);
    ros::Subscriber im_sub = n.subscribe("/fused_depth",1000,imCallback);
    ros::Rate loop_rate(10);

    while (ros::ok()){
        ros::spinOnce();
    }
    
    return 0;
}

void imCallback(const std_msgs::Float64MultiArray::ConstPtr& im_msg){
    
    im.set_size(im_msg->layout.dim[0].size, im_msg->layout.dim[1].size);
   
    // msg->layout.dim[0] -> number of rows
    // msg->layout.dim[1] -> number of columns 
    for (int i = 0; i < im_msg->layout.dim[0].size; ++i){
        for (int j = 0; j < im_msg->layout.dim[1].size; ++j){    
             
            im(i,j) = im_msg->data[im_msg->layout.dim[1].stride*i + j]; //im_msg->layout.data_offset + 
        
        }
    }
    
    std::cout << "matrix recieved" << std::endl;
   
        
    // Create 3D tensor (stack of image sized matrices with each slice corresponding to a different radius
    // H.set_size(im.n_rows, im.n_cols, radius);
    H.zeros(im.n_rows, im.n_cols, radius_max);

    // Create H index variables
    double a =0, b = 0;

    arma::uvec edge = find(im != 0);
    arma::vec edges = arma::conv_to<arma::vec>::from(edge);
    edges += 1;
    arma::vec edge_x = arma::zeros<arma::vec>(edges.n_elem);
    arma::vec edge_y = arma::zeros<arma::vec>(edges.n_elem);

    // Loop over extracted edge pixels
    for (int l = 0; l < edges.n_elem; ++l){
        edge_x(l) = fmod(edges(l), double(im.n_rows)) - 1;
        edge_y(l) = floor(edges(l) / im.n_rows);
        if (fmod(edges(l), double(im.n_rows)) == 0){
        	edge_x(l) = double(im.n_rows) - 1;
        	edge_y(l) -= 1;
        }
        if (edges(l) == 1){
        	edge_x(l) = 0;
        	edge_y(l) = 0;
        }

        // edge_x.print();
        // edge_y.print();

        // Loop over radii
        for (int r = radius_min - 1; r < radius_max; ++r){
            // loop over thetas
            for (int t = 0; t < theta; ++t){
                a = edge_x(l) - r * cos(theta * 0.0174533);
                b = edge_y(l) + r * sin(theta * 0.0174533);

                if (round(a) < H.n_rows && round(b) < H.n_cols && round(a) >= 0 && round(b) >= 0){
                    H(round(a), round(b), r) += 1;
                }
            }
        }
    }

    // Initialize containers for extracted centers
    arma::uvec center_r;
    arma::uvec all_center_indices;
    arma::vec r_r;
    arma::vec all_r;
    arma::vec center_x;
    arma::vec center_y;
    arma::vec votes;

    arma::mat center_indices;           // to be sorted left -> right -> top -> bottom
    arma::mat center_indices_vote;      // to be sorted by number of votes

    for (int slice = radius_min - 1; slice < radius_max; ++slice){
        
        // extract only local maxima which are circle centers
        double thresh = 0.9 * arma::max(arma::max(H.slice(slice)));
        H.slice(slice).elem(arma::find(H.slice(slice) < thresh)).zeros();
        
        // transpose H slice so that find will traverse original values left -> right -> top -> bottom
        // TODO -> this needs to be its own matrix, otherwise when you pull the values for votes vector they will be wrong
        H.slice(slice).t(); 
        
        // find center locations (columnwise on transposed slice corresponds to l -> r -> t -> b in original slice)
        center_r = arma::find(H.slice(slice) != 0);
        
        // create another vector with radii
        r_r.zeros(center_r.n_elem);
        r_r += slice; // don't forget that this is the smallest r value, set in for loop initializer above

        //edges = arma::conv_to<arma::vec>::from(edge);
        //center_r += 1;
       
        // "push back" centers and radii from the current slice
        all_center_indices.resize(all_center_indices.n_elem + center_r.n_elem);
        all_r.resize(all_r.n_elem + r_r.n_elem);
        all_center_indices.subvec(all_center_indices.n_elem - center_r.n_elem, all_center_indices.n_elem - 1) = center_r;    
        all_r.subvec(all_r.n_elem - r_r.n_elem, all_r.n_elem - 1) = r_r;
    
        // REMOVED COMMENTED SECTION AND PUT IN SCATCH.TXT (DESKTOP)

    }
    
    // initialize vectors for x and y indices of circle centers
    center_x.zeros(all_center_indices.n_elem);
    center_y.zeros(all_center_indices.n_elem);
    votes.zeros(all_center_indices.n_elem); 

    // order all centers in left -> right -> top -> bottom order, apply to radii as well
    // NOTE: each center_r in the loop over slices was in correct order but referenced the 
    // transposed slice... here the full set of indices referencing the transposed vector is
    // sorted, then x y indices will be extracted, and finally x and y locations will be swapped,
    // effectively re-referencing the original slice pixel locations before the transpose
    all_r = all_r(arma::sort_index(all_center_indices));
    all_center_indices = arma::sort(all_center_indices);
    
    // convert to doubles for arithmetic operations
    arma::vec all_center_index = arma::conv_to<arma::vec>::from(all_center_indices);
    
    // extract x and y pixel locations (remember these are reversed because of the transpose)
    all_center_index += 1;

    for (int l = 0; l < all_center_index.n_elem; ++l){
        center_x(l) = fmod(all_center_index(l), double(im.n_rows)) - 1;
        center_y(l) = floor(all_center_index(l) / im.n_rows);
        if (fmod(all_center_index(l), double(im.n_rows)) == 0){
            center_x(l) = double(im.n_rows) - 1;
            center_y(l) -= 1;
        }
        if (all_center_index(l) == 1){
            center_x(l) = 0;
            center_y(l) = 0;
        }
    }
   
    // fill output list of centers and radii
    // NOTE: X AND Y ARE SWITCHED BECAUSE THE INDICES WERE ORIGINALLY GENERATED FROM THE TRANSPOSE OF THE SLICE 
    center_indices.set_size(center_x.n_elem, 3);
    center_indices_vote.set_size(center_x.n_elem, 3);

    center_indices.col(0) = center_y;
    center_indices.col(1) = center_x;
    center_indices.col(2) = all_r;
    
    for (int vote_sort = 0; vote_sort < votes.n_elem; ++vote_sort){
        votes(vote_sort) = H(center_indices(vote_sort, 0), center_indices(vote_sort, 1), center_indices(vote_sort, 2)); 
    }
    
    arma::vec transfer_vec;

    transfer_vec = center_indices.col(0);
    center_indices_vote.col(0) = transfer_vec(arma::sort_index(votes));
    
    transfer_vec = center_indices.col(1);
    center_indices_vote.col(1) = transfer_vec(arma::sort_index(votes));

	transfer_vec = center_indices.col(2);
    center_indices_vote.col(2) = transfer_vec(arma::sort_index(votes));
 
    // create message when ready to publish
	std_msgs::Float64MultiArray mat_msg;
	mat_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	mat_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	//mat_msg.layout.data_offset = 32;
	mat_msg.layout.dim[0].label = "rows";
	// number of rows
	mat_msg.layout.dim[0].size = center_indices.n_rows;
	//number of rows * number of columns
	mat_msg.layout.dim[0].stride = center_indices.n_rows*center_indices.n_cols;
	mat_msg.layout.dim[1].label = "columns";
	// number of columns
	mat_msg.layout.dim[1].size = center_indices.n_cols;
	mat_msg.layout.dim[1].stride = center_indices.n_cols;

	// loop over row indices and column indices
	for (int rows = 0; rows < mat_msg.layout.dim[0].size; ++rows){
		for (int cols = 0; cols < mat_msg.layout.dim[1].size; ++cols){

			// push back each matrix value
			mat_msg.data.push_back(center_indices(rows, cols));

		}
	}

    // message sorted by votes
    std_msgs::Float64MultiArray mat_msg2;
	mat_msg2.layout.dim.push_back(std_msgs::MultiArrayDimension());
	mat_msg2.layout.dim.push_back(std_msgs::MultiArrayDimension());
	//mat_msg2.layout.data_offset = 32;
	mat_msg2.layout.dim[0].label = "rows";
	// number of rows
	mat_msg2.layout.dim[0].size = center_indices_vote.n_rows;
	//number of rows * number of columns
	mat_msg2.layout.dim[0].stride = center_indices_vote.n_rows*center_indices_vote.n_cols;
	mat_msg2.layout.dim[1].label = "columns";
	// number of columns
	mat_msg2.layout.dim[1].size = center_indices_vote.n_cols;
	mat_msg2.layout.dim[1].stride = center_indices_vote.n_cols;

	// loop over row indices and column indices
	for (int rows = 0; rows < mat_msg2.layout.dim[0].size; ++rows){
		for (int cols = 0; cols < mat_msg2.layout.dim[1].size; ++cols){

			// push back each matrix value
			mat_msg2.data.push_back(center_indices_vote(rows, cols));

		}
	}



	// publish message
	mat_pub.publish(mat_msg);
    mat_pub2.publish(mat_msg2);
}

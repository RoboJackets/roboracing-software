for(int i=0;i < 2*radius;i++)
	{
		float hsize=pow(pow(radius-i,2)+pow(radius,2),(1/2));
		for (int j = 0; j < hsize; j++)
		{
		     if (img.at<uchar>(i,j) > 240)
		     {
		     	goodPoints++;

		     }
		     totalPoints++;
		     //ROS_INFO("%d",img.at<uchar>(i,j));
		}	
	}
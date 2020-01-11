#include "depth_render.cuh"
__global__ void render(float3 *data_devptr, Parameter *para_devptr, DeviceImage<int> *depth_devptr)
{
	const int index = threadIdx.x + blockIdx.x * blockDim.x;
	const Parameter para = *para_devptr;
	if(index >= para.point_number)
		return;
	float3 my_point = data_devptr[index];
	
	//transform
	float3 trans_point;
	trans_point.x = my_point.x * para.r[0][0] + my_point.y * para.r[0][1] + my_point.z * para.r[0][2] + para.t[0];
	trans_point.y = my_point.x * para.r[1][0] + my_point.y * para.r[1][1] + my_point.z * para.r[1][2] + para.t[1];
	trans_point.z = my_point.x * para.r[2][0] + my_point.y * para.r[2][1] + my_point.z * para.r[2][2] + para.t[2];

	if(trans_point.z <= 0.0f)
		return;

	//project
	int2 projected;
	projected.x = trans_point.x / trans_point.z * para.fx + para.cx + 0.5;
	projected.y = trans_point.y / trans_point.z * para.fy + para.cy + 0.5;
	if(projected.x < 0 || projected.x >= para.width || projected.y < 0 || projected.y >= para.height)
		return;

	// float dist = length(trans_point);
	float dist = trans_point.z;
	int dist_mm = dist * 1000.0f + 0.5f;

	//int r = 0.0173 * para.fx / dist + 0.5f;
//	int r = 0.0473 * para.fx / dist + 0.5f;
	int r = 0.0573 * para.fx / dist + 0.5f;
	for(int i = -r; i <= r; i++)
		for(int j = -r; j <= r; j++)
		{
			int to_x = projected.x + j;
			int to_y = projected.y + i;
			if(to_x < 0 || to_x >= para.width || to_y < 0 || to_y >= para.height)
				continue;
			int *dist_ptr = &(depth_devptr->atXY(to_x, to_y));
			atomicMin(dist_ptr, dist_mm);
		}
}

__global__ void depth_initial(DeviceImage<int> *depth_devptr)
{
  const int x = threadIdx.x + blockIdx.x * blockDim.x;
  const int y = threadIdx.y + blockIdx.y * blockDim.y;
  int width = depth_devptr->width;
  int height = depth_devptr->height;

  if(x >= width || y >= height)
  	return;

  depth_devptr->atXY(x,y) = 999999;
}

DepthRender::DepthRender():
	cloud_size(0),
	host_cloud_ptr(NULL),
	dev_cloud_ptr(NULL),
	has_devptr(false)
{
}

DepthRender::~DepthRender()
{
	if(has_devptr)
	{
		free(host_cloud_ptr);
		cudaFree(dev_cloud_ptr);
		cudaFree(parameter_devptr);
	}
}

void DepthRender::set_para(float _fx, float _fy, float _cx, float _cy, int _width, int _height)
{
	parameter.fx = _fx;
	parameter.fy = _fy;
	parameter.cx = _cx;
	parameter.cy = _cy;
	parameter.width = _width;
	parameter.height = _height;
}

void DepthRender::set_data(vector<float> &cloud_data)
{
	cloud_size = cloud_data.size() / 3;
	parameter.point_number = cloud_size;

	host_cloud_ptr = (float3*) malloc(cloud_size * sizeof(float3));
	for(int i = 0; i < cloud_size; i++)
		host_cloud_ptr[i] = make_float3(cloud_data[3*i], cloud_data[3*i+1], cloud_data[3*i+2]);

  cudaError err = cudaMalloc(&dev_cloud_ptr, cloud_size * sizeof(float3));
  if(err != cudaSuccess)
    throw CudaException("DeviceLinear: unable to allocate linear memory.", err);
 	err = cudaMemcpy(
          dev_cloud_ptr,
          host_cloud_ptr,
          cloud_size * sizeof(float3),
          cudaMemcpyHostToDevice);
  if(err != cudaSuccess)
  	throw CudaException("DeviceLinear: unable to copy data from host to device.", err);

  err = cudaMalloc(&parameter_devptr, sizeof(Parameter));
  if(err != cudaSuccess)
    throw CudaException("DeviceLinear: unable to allocate linear memory.", err);
 	err = cudaMemcpy(
          parameter_devptr,
          &parameter,
          sizeof(Parameter),
          cudaMemcpyHostToDevice);
  if(err != cudaSuccess)
  	throw CudaException("DeviceLinear: unable to copy data from host to device.", err);

  has_devptr = true;

  //printf("load points done!\n");
}

/*void DepthRender::render_pose( Matrix3d &rotation, Vector3d &translation, int *host_ptr)
{
	for(int i = 0; i < 3; i++)
	{
		parameter.t[i] = translation(i);
		for(int j = 0; j < 3; j++)
		{
			parameter.r[i][j] = rotation(i,j);
		}
	}
 	cudaError err = cudaMemcpy(
          parameter_devptr,
          &parameter,
          sizeof(Parameter),
          cudaMemcpyHostToDevice);
  if(err != cudaSuccess)
  	throw CudaException("DeviceLinear: unable to copy data from host to device.", err);

	DeviceImage<int> depth_output(parameter.width, parameter.height);
  depth_output.zero();

  dim3 depth_block;
  dim3 depth_grid;
  depth_block.x = 16;
  depth_block.y = 16;
  depth_grid.x = (parameter.width + depth_block.x - 1 ) / depth_block.x;
  depth_grid.y = (parameter.height + depth_block.y - 1 ) / depth_block.y;
  depth_initial<<<depth_grid, depth_block>>>(depth_output.dev_ptr);

  dim3 render_block;
  dim3 render_grid;
  render_block.x = 64;
  render_grid.x = (cloud_size + render_block.x - 1) / render_block.x;
  render<<<render_grid, render_block>>>(dev_cloud_ptr, parameter_devptr, depth_output.dev_ptr);

	depth_output.getDevData(host_ptr);
}
*/
//void DepthRender::render_pose( Matrix4d &transformation, int *host_ptr)
void DepthRender::render_pose( double * transformation, int *host_ptr)
{
	for(int i = 0; i < 3; i++)
	{
		parameter.t[i] = transformation[4 * i + 3];//transformation(i,3);
		for(int j = 0; j < 3; j++)
		{
			parameter.r[i][j] = transformation[4 * i + j];//transformation(i,j);
		}
	}
 	cudaError err = cudaMemcpy(
          parameter_devptr,
          &parameter,
          sizeof(Parameter),
          cudaMemcpyHostToDevice);
  if(err != cudaSuccess)
  	throw CudaException("DeviceLinear: unable to copy data from host to device.", err);

	DeviceImage<int> depth_output(parameter.width, parameter.height);
  depth_output.zero();

  dim3 depth_block;
  dim3 depth_grid;
  depth_block.x = 16;
  depth_block.y = 16;
  depth_grid.x = (parameter.width + depth_block.x - 1 ) / depth_block.x;
  depth_grid.y = (parameter.height + depth_block.y - 1 ) / depth_block.y;
  depth_initial<<<depth_grid, depth_block>>>(depth_output.dev_ptr);

  dim3 render_block;
  dim3 render_grid;
  render_block.x = 64;
  render_grid.x = (cloud_size + render_block.x - 1) / render_block.x;
  render<<<render_grid, render_block>>>(dev_cloud_ptr, parameter_devptr, depth_output.dev_ptr);

	depth_output.getDevData(host_ptr);
}
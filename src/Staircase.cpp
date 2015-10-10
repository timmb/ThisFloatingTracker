#include "Staircase.h"
#include "cinder/app/App.h"
#include "Common.h"

using namespace std;
using namespace ci;

class StaircaseShader : public Shader
{
public:
	StaircaseShader(Staircase* staircase_)
		: Shader("Staircase", "Staircase", "frag_staircase.glsl", "vert_staircase.glsl")
		, staircase(staircase_)
	{

	}

	Staircase* staircase;

protected:
	virtual void setUniforms() override
	{
		program->uniform("w", staircase->mNumVerticesX);
		program->uniform("h", staircase->mNumVerticesY);

		UserStats stats = gInput->userStats.getUserStats();


		program->uniform("cumulativeQom", stats.qomWithDecay);
		program->uniform("userVel", stats.centroidVel);

	}
};

Staircase::Staircase()
: Sketch("Staircase")
, mNumVerticesX(20)
, mNumVerticesY(20)
{
	pars.addParam(new Parameter<int>(&mNumVerticesX, "mNumVerticesX", { "Staircase" }));
	pars.addParam(new Parameter<int>(&mNumVerticesY, "mNumVerticesY", { "Staircase" }));
}

void Staircase::setup()
{
	mShader = ShaderPtr(new StaircaseShader(this));
	setupVbo();
}

void Staircase::recompileShaders()
{
	mShader->compile();
}

void Staircase::setupVbo()
{
	int totalVertices = mNumVerticesX * mNumVerticesY;
	int totalQuads = (mNumVerticesX - 1) * (mNumVerticesY - 1);
	gl::VboMesh::Layout layout;
	layout.setStaticIndices();
	layout.setStaticPositions();
	layout.setStaticTexCoords2d();

	mVbo = gl::VboMesh::create(totalVertices, totalQuads * 4, layout, GL_QUADS);

	// generate vertex positions
	vector<Vec3f> positions;
	//gl::VboMesh::VertexIter iter = mVbo->mapVertexBuffer();
	for (int x = 0; x < mNumVerticesX; ++x) {
		for (int y = 0; y < mNumVerticesY; ++y) {
			// grid in normalized coord space
			//iter.setPosition(Vec3f(float(x) / (mNumVerticesX - 1) * 2.f - 1.f, float(y) / (mNumVerticesY - 1) * 2.f - 1.f, 0.f));
			positions.push_back(Vec3f(float(x) / (mNumVerticesX - 1) * 2.f - 1.f, float(y) / (mNumVerticesY - 1) * 2.f - 1.f, 0.f));

			//float height = sin(y / (float)mNumVerticesY * zFreq + x / (float)mNumVerticesX * xFreq + offset) / 5.0f;
			//iter.setPosition(Vec3f(x / (float)mNumVerticesX, height, y / (float)mNumVerticesY));
			//++iter;
		}
	}
	mVbo->bufferPositions(positions);

	//  the texcoords and the indices
	vector<uint32_t> indices;
	vector<Vec2f> texCoords;
	for (int x = 0; x < mNumVerticesX; ++x) {
		for (int y = 0; y < mNumVerticesY; ++y) {
			// create a quad for each vertex, except for along the bottom and right edges
			if ((x + 1 < mNumVerticesX) && (y + 1 < mNumVerticesY)) {
				indices.push_back((x + 0) * mNumVerticesY + (y + 0));
				indices.push_back((x + 1) * mNumVerticesY + (y + 0));
				indices.push_back((x + 1) * mNumVerticesY + (y + 1));
				indices.push_back((x + 0) * mNumVerticesY + (y + 1));
			}
			// the texture coordinates are mapped to [0,1.0)
			texCoords.push_back(Vec2f(x / (float)(mNumVerticesX - 1), y / (float)(mNumVerticesY - 1)));
		}
	}



	mVbo->bufferIndices(indices);
	mVbo->bufferTexCoords2d(0, texCoords);

	mVbo->unbindBuffers();
}

void Staircase::update()
{
	if (app::getElapsedFrames() < 5)
		setupVbo();
}

void Staircase::draw()
{
	mShader->bind();

	glDisable(GL_TEXTURE_2D);
	gl::enableAdditiveBlending();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gl::clear();
	//gl::clear(ColorA(1, 0, 1, 1));
	gl::color(ColorA(0.5, 1., 1., 1.));
	gl::draw(mVbo);

	mShader->unbind();
}
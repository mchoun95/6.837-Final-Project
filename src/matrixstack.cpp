#include "matrixstack.h"

MatrixStack::MatrixStack()
{
	// Initialize the matrix stack with the identity matrix.
	Matrix4f I = Matrix4f::identity();
	m_matrices.push_back(I);
}

void MatrixStack::clear()
{
	// Revert to just containing the identity matrix.
	m_matrices.clear();
	Matrix4f I = Matrix4f::identity();
	m_matrices.push_back(I);
}

Matrix4f MatrixStack::top()
{
	// Return the top of the stack
	// return Matrix4f();

    return m_matrices.back();
}

void MatrixStack::push( const Matrix4f& m )
{
	// Push m onto the stack.
	// The new top should be "old * m", so that conceptually the new matrix
    // is applied first in right-to-left evaluation.
	Matrix4f top = m_matrices.back();
	m_matrices.push_back(top * m);
}

void MatrixStack::pop()
{
	// Remove the top element from the stack
	m_matrices.pop_back();
}

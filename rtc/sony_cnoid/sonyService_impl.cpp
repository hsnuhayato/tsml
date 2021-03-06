// -*-C++-*-
/*!
 * @file  sonyService_impl.cpp
 * @brief Service implementation code of sonyService.idl
 *
 */

#include "sonyService_impl.h"

/*
 * Example implementational code for IDL interface OpenHRP::sonyService
 */
sonyService_impl::sonyService_impl()
{
  // Please add extra constructor code here.
}


sonyService_impl::~sonyService_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void sonyService_impl::start()
{
  m_comp->start();
}

void sonyService_impl::setObjectV(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z, ::CORBA::Double roll, ::CORBA::Double pitch, ::CORBA::Double yaw)
{
  m_comp->setObjectV(x, y, z, roll, pitch, yaw);
}

void sonyService_impl::stepping()
{
  m_comp->stepping();
}

void sonyService_impl::testMove()
{
  m_comp->testMove();
}

void sonyService_impl::setFootPosR2()
{
  m_comp->setFootPosR();
}

void sonyService_impl::setFootPosL2()
{
  m_comp->setFootPosL();
}


void sonyService_impl::stop()
{
  m_comp->stop();
}

void sonyService_impl::freeWalkSwitch()
{
  m_comp->freeWalkSwitch();
}

void sonyService_impl::freeWalkSwitchOn()
{
  m_comp->freeWalkSwitchOn();
}

void sonyService_impl::freeWalkSwitchOff()
{
  m_comp->freeWalkSwitchOff();
}

void sonyService_impl::setFootPosR(double x, double y, double z, double r, double p, double w)
{
  m_comp->setFootPosR(x,y,z,r,p,w);
}

void sonyService_impl::setFootPosL(double x, double y, double z, double r, double p, double w)
{
  m_comp->setFootPosL(x,y,z,r,p,w);
}

// ogawa
void sonyService_impl::logStart(const char *date)
{
  m_comp->logStart(date);
}

// End of example implementational code




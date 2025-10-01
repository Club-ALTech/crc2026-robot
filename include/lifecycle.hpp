#pragma once

/**
 * this interface represents something that follows arduino's typical lifecycle such as sensors, IO pins, etc.
 * 
 * expected use: 
 * - inject dependencies through the constructor. see: 
 * - implement lifecycle methods
 *   - begin should be called in the setup()
 *   - update should be called in the loop()
 */
class ILifecycle
{
private:
public:
    virtual ~ILifecycle() {}
    virtual void begin() = 0;
    virtual void update() = 0;
};

#ifndef EXPM_OPTIONS_H
#define EXPM_OPTIONS_H

enum class ExpmTarget
{
    SCENE,
    OBJECT,
    SAMPLE
};

struct ExpmUiOptions
{
    ExpmTarget target;
    bool writeDesc;
};

#endif // EXPM_OPTIONS_H

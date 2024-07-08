using System;
using System.Runtime.InteropServices;
using UnityEngine;

public class ForceFeedbackExample : MonoBehaviour
{
    private const uint SDL_INIT_JOYSTICK = 0x00000200;
    private const uint SDL_INIT_HAPTIC = 0x00001000;

    [DllImport("libSDL2.so", EntryPoint = "SDL_Init", CallingConvention = CallingConvention.Cdecl)]
    public static extern int SDL_Init(uint flags);

    [DllImport("libSDL2.so", EntryPoint = "SDL_Quit", CallingConvention = CallingConvention.Cdecl)]
    public static extern void SDL_Quit();

    [DllImport("libSDL2.so", EntryPoint = "SDL_JoystickOpen", CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr SDL_JoystickOpen(int device_index);

    [DllImport("libSDL2.so", EntryPoint = "SDL_JoystickClose", CallingConvention = CallingConvention.Cdecl)]
    public static extern void SDL_JoystickClose(IntPtr joystick);

    [DllImport("libSDL2.so", EntryPoint = "SDL_HapticOpenFromJoystick", CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr SDL_HapticOpenFromJoystick(IntPtr joystick);

    [DllImport("libSDL2.so", EntryPoint = "SDL_HapticRumbleInit", CallingConvention = CallingConvention.Cdecl)]
    public static extern int SDL_HapticRumbleInit(IntPtr haptic);

    [DllImport("libSDL2.so", EntryPoint = "SDL_HapticRumblePlay", CallingConvention = CallingConvention.Cdecl)]
    public static extern int SDL_HapticRumblePlay(IntPtr haptic, float strength, uint length);

    [DllImport("libSDL2.so", EntryPoint = "SDL_HapticClose", CallingConvention = CallingConvention.Cdecl)]
    public static extern void SDL_HapticClose(IntPtr haptic);

    void Start()
    {
        if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC) < 0)
        {
            Debug.LogError("Failed to initialize SDL: " + Marshal.PtrToStringAnsi(SDL_GetError()));
            return;
        }

        int joystickIndex = 0;
        IntPtr joystick = SDL_JoystickOpen(joystickIndex);
        if (joystick == IntPtr.Zero)
        {
            Debug.LogError("Failed to open joystick: " + Marshal.PtrToStringAnsi(SDL_GetError()));
            return;
        }

        IntPtr haptic = SDL_HapticOpenFromJoystick(joystick);
        if (haptic == IntPtr.Zero)
        {
            Debug.LogError("Failed to open haptic: " + Marshal.PtrToStringAnsi(SDL_GetError()));
            return;
        }

        if (SDL_HapticRumbleInit(haptic) < 0)
        {
            Debug.LogError("Failed to initialize haptic rumble: " + Marshal.PtrToStringAnsi(SDL_GetError()));
            return;
        }

        SDL_HapticRumblePlay(haptic, 0.75f, 2000);  // Play rumble at 75% strength for 2000ms

        SDL_HapticClose(haptic);
        SDL_JoystickClose(joystick);
        SDL_Quit();
    }

    [DllImport("libSDL2.so", EntryPoint = "SDL_GetError", CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr SDL_GetError();
}
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Player_movement : MonoBehaviour
{
    public float speed = 6.0f;                // Normal movement speed
    public float sprintSpeed = 12.0f;         // Sprinting speed
    public float gravity = -9.8f;             // Gravity force
    public float jumpHeight = 1.5f;           // Jump height
    public float mouseSensitivity = 100.0f;   // Mouse sensitivity
    public Transform cameraTransform;         // Reference to the player's camera

    private CharacterController controller;
    private Vector3 velocity;
    private bool isGrounded;

    private float verticalRotation = 0f;      // For camera vertical rotation

    void Start()
    {
        // Get the Character Controller component
        controller = GetComponent<CharacterController>();

        // Lock the cursor to the center of the screen and hide it
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }

    void Update()
    {
        // Handle mouse movement for looking around
        HandleMouseLook();

        // Handle player movement, jumping, and sprinting
        HandleMovement();
    }

    void HandleMouseLook()
    {
        // Get the mouse input
        float mouseX = Input.GetAxis("Mouse X") * mouseSensitivity * Time.deltaTime;
        float mouseY = Input.GetAxis("Mouse Y") * mouseSensitivity * Time.deltaTime;

        // Rotate the player (horizontal rotation)
        transform.Rotate(Vector3.up * mouseX);

        // Apply vertical camera rotation (up and down), clamping the value to avoid flipping
        verticalRotation -= mouseY;
        verticalRotation = Mathf.Clamp(verticalRotation, -90f, 90f); // Clamp to prevent over-rotation

        // Apply the rotation to the camera
        cameraTransform.localRotation = Quaternion.Euler(verticalRotation, 0f, 0f);
    }

    void HandleMovement()
    {
        // Check if the player is grounded
        isGrounded = controller.isGrounded;
        if (isGrounded && velocity.y < 0)
        {
            velocity.y = -2f; // Keep the player grounded
        }

        // Determine the movement speed (normal or sprinting)
        float currentSpeed = Input.GetKey(KeyCode.LeftShift) ? sprintSpeed : speed;

        // Get input for movement (WASD or arrow keys)
        float moveX = Input.GetAxis("Horizontal");
        float moveZ = Input.GetAxis("Vertical");

        // Move the player based on the input
        Vector3 move = transform.right * moveX + transform.forward * moveZ;
        controller.Move(move * currentSpeed * Time.deltaTime);

        // Handle jumping
        if (isGrounded && Input.GetButtonDown("Jump"))
        {
            velocity.y = Mathf.Sqrt(jumpHeight * -2f * gravity);
        }

        // Apply gravity
        velocity.y += gravity * Time.deltaTime;
        controller.Move(velocity * Time.deltaTime);
    }
}

# Coding standards

This guide is the coding standard used by AWSIM. Inspired by [C# at Google Style Guide](https://google.github.io/styleguide/) 
and [Unity's CREATE A C# STYLE GUIDE](https://unity.com/resources/create-code-c-sharp-style-guide-e-book) and created with some modifications.
Basically, please follow these guidelines for coding.
## C# Formatting guidelines

### Code
- Names of classes, methods, enumerations, public fields, public properties, namespaces: `PascalCase`.
- Names of private, protected, internal and protected internal fields and properties: `_camelCase`. (Use underscore as prefix)
- Names of local variables, parameters: `camelCase`. (Not use underscore as prefix)
- Naming convention is unaffected by modifiers such as const, static, readonly, etc.
- For casing, a “word” is anything written without internal spaces, including acronyms. For example, `MyRpc` instead of ~~`MyRPC`~~.
- Names of interfaces start with `I`, e.g. `IInterface`.

### Files
- Filenames and directory names are `PascalCase`, e.g. `MyFile.cs`.
- Where possible the file name should be the same as the name of the main class in the file, e.g. `MyClass.cs`.
- In general, prefer one core class per file.

### Organization
- Modifiers occur in the following order: `public protected internal private new abstract virtual override sealed static readonly extern unsafe volatile async`.
- `private` attribute is omitted.
- Declare using namespaces in order of decreasing dependency, starting from the top. 
- Class member ordering:
    - Group class members in the following order:
        - Nested classes, enums, delegates and events.
        - Static, const and readonly fields.
        - Fields and properties.
        - Constructors and finalizers.
        - Methods.
    - Within each group, elements should be in the following order:
        - Public.
        - Internal.
        - Protected internal.
        - Protected.
        - Private.
    - Where possible, group interface implementations together.

### Whitespace rules

- A maximum of one statement per line.
- A maximum of one assignment per statement.
- Indentation of 4 spaces, no tabs.
- Column limit: around 140. (Not strictly)
- Yes line break before opening brace.
- Yes line break between closing brace and `else`.
- Space after `if`/`for`/`while` etc., and after commas.
- No space after an opening parenthesis or before a closing parenthesis.
- No space between a unary operator and its operand. One space between the
  operator and each operand of all other operators.
- The following rules apply to line wrapping.
    - In general, line continuations are indented 4 spaces.
    - Line breaks with braces (e.g. list initializers, lambdas, object
      initializers, etc) do not count as continuations.
    - For function definitions and calls, if the arguments do not all fit on
      one line they should be broken up onto multiple lines, with each
      subsequent line aligned with the first argument. If there is not enough
      room for this, arguments may instead be placed on subsequent lines with
      a four space indent. The code example below illustrates this.

### Example code

```cs {.yaml .no-copy}
using System;                   // System is a primitive namespace in C#.
using System.Collections;
using UnityEngine;              // UnityEngine is a C#-dependent namespace.
using UnityEngine.UI;
using ROS2;                     // ROS2 is an external ROS2 library available 
                                // in Unity.
using Awsim.EgoVehicle;         
using Awsim.General;

namespace Awsim.Sample.SampleNameScope
{
    // Write XML comments in the public attribute.
    /// <summary>
    /// XML comments.                                                
    /// </summary>
    public interface ISampleInterface // Interfaces start with 'I'
    {
        // Methods are PascalCase ...and space after comma.
        public int Calculate(float value, float exp);
    }

    /// <summary>
    /// XML comments.
    /// </summary>
    public enum SampleEnum      // Enumerations are PascalCase.
    {
        Yes,                    // Enumerators are PascalCase.
        No,
    }

    /// <summary>
    /// XML comments.
    /// </summary>
    public class SampleClass    // Classes are PascalCase.
    {
        // Group class members in the following order:
        //     1. Nested classes, enums, delegates and events.
        //     2. Static, const and readonly fields.
        //     3. Properties and fields.
        //     4. Constructors and finalizers.
        //     5. Methods.

        /// <summary>
        /// XML comments.
        /// </summary>                                              
        // Within the same group, those with public 
        // attributes are described higher.
        public class InnerPublicClass
        {
            /// <summary>
            /// XML comments.
            /// </summary>
            // Public member variables are PascalCase.
            // Public member variables are basically properties.
            // Field initializers are encouraged.
            public int Foo { get; set; } = 0;           

            /// <summary>
            /// XML comments.
            /// </summary>
            // For single line read-only properties, 
            // prefer expression body properties (=>) when possible.
        }
            public int Bar => Foo;                                  

        // Private attributes are omitted.
        class InnerPrivateClass
        {

        }

        /// <summary>
        /// XML comments.
        /// </summary>
        // const does not affect naming convention.
        public const string ConstKey = "ConstKey";                  

        // protected member variables are _camelCase.
        protected const string _constKey2 = "ConstKey2";             
            
        // Private member variables are _camelCase.
        InnerPublicClass _class = null;

        int _value = 0;                                             

        // Arrays are initialized on a single line, if possible.
        int[] _someShortTable = { 2, 3, 4, };              

        // If the element is long, break lines.
        int[] _someLongTable = new int[3]                           
        {
            2,
            3,
            4,
        };

        // Matrices, etc. may break lines for easier comprehension.
        int[] _matrix = new int[9]                                  
        {
            0, 1, 2,
            3, 4, 5,
            6, 7, 8
        };

        // String uses an empty string instead of null.
        string _sampleStr = String.Empty;

        /// <summary>
        /// XML comments.
        /// </summary>
        public SampleClass()
        {
            _class = new InnerPublicClass
            {
                // Object initializer use a 4 space indent.
                Foo = 1,                                            
            };
        }

        /// <summary>
        /// XML comments.
        /// </summary>
        // No line break before opening brace.
        public int Add(int addValue, bool useLog)                   
        {
            // Local variables are camelCase.
            var resultValue = _value + addValue;                    

            if (useLog)
            {
                var sign = MathF.Sign(resultValue);
                var isMinusValue = sign < 0;

                // No space after unary operator and space after 'if'.
                if (!isMinusValue)                                  
                {
                    Log("Sum is minus value : ", resultValue);
                }
                else
                {
                    Log("Sum is plus value : ", resultValue);
                }
            }

            return resultValue;

            // Use of local functions is recommended.
            void Log(string prefix, int value)                      
            {
                Debug.Log(prefix + value);
            }
        }

        /// <summary>
        /// XML comments.
        /// </summary>
        public void ExpressionBodies()
        {
            // For simple lambdas, fit on one line if possible, 
            // no brackets or braces required.
            Func<int, int> increment = x => x + 1;

            // A block of lambda expressions is written on a new line.
            Func<int, int, long> difference1 = (x, y) =>
            {
                long diff = (long)x - y;
                return diff >= 0 ? diff : -diff;
            };
        }

        void DoNothing() { }    // Empty blocks may be concise.

        // If possible, wrap arguments 
        // by aligning newlines with the first argument.
        void LongFunctionNameThatCausesLineWrappingProblems(int ArgumentName,
                                                            int p1, 
                                                            int p2)
        {

        }

        // If aligning argument lines with the first argument doesn't fit, 
        // or is difficult to read, wrap all arguments 
        // on new lines with a 4 space indent.
        void AnotherLongFunctionNameThatCausesLineWrappingProblems(
            int longArgumentName, int longArgumentName2, int longArgumentName3)
        {

        }

        void CallingLongFunctionName()
        {
            int veryLongArgumentName = 1234;
            int shortArg = 1;
            // If possible, wrap arguments 
            // by aligning newlines with the first argument.
            LongFunctionNameThatCausesLineWrappingProblems(shortArg, 
                                                           shortArg,
                                                           LongArgumentName);


            // If aligning argument lines with the first argument doesn't fit, 
            // or is difficult to read, 
            // wrap all arguments on new lines with a 4 space indent.
            LongFunctionNameThatCausesLineWrappingProblems(
                veryLongArgumentName, veryLongArgumentName, LongArgumentName);
        }
    }
}
```

## C# coding guidelines

### Comments
- Create an XML comment for the public attribute. (It is automatically created by typing `///` in Visual studio or VSCode).
- Writing XML comments in the inner class.
- Beginning with a capital letter and ending with a period. `// This is sample comment.`
  ```cs
  /// <summary>
  /// XML comments.
  /// </summary>
  public class SampleClass
  {
      /// <summary>
      /// XML comments.
      /// </summary>
      class InnerClass
      {

          /// <summary>
          /// XML comments.
          /// </summary>
          public void SampleMethod()
          {

          }
      }

  }
  ```

### Constants

- Variables and fields that can be made `const` should always be made `const`.
- If `const` isn’t possible, `readonly` can be a suitable alternative.
- Prefer named constants to magic numbers.

### Property styles

- For single line read-only properties, prefer expression body properties
    (`=>`) when possible.
- Also use (`=>`) to simply be a getter or setter of a local field. `{ get => variables; set => variables = value; }`
- For everything else, use the older `{ get; set; }` syntax.

### Structs and classes:

- Structs are very different from classes:
    - Structs are always passed and returned by value.
    - Assigning a value to a member of a returned struct doesn’t modify the
        original - e.g. `transform.position.x = 10` doesn’t set the transform’s
        position.x to 10; `position` here is a property that returns a `Vector3`
        by value, so this just sets the x parameter of a copy of the original.
- Consider struct when the type can be treated like other value types - for
  example, if instances of the type are small and commonly short-lived or are
  commonly embedded in other objects. Good examples include Vector3,
  Quaternion and Bounds.
- Be careful about “new” classes and structures.
    - For example, Unity's MonoBehaviour's Update() and FixedUpdate() can negatively 
      impact performance if a new class or structure is created for each frame.
    - Be especially careful with the new class.

### Field initializers

- Field initializers are generally encouraged.

### Extension methods

- Only use an extension method when the source of the original class is not
  available, or else when changing the source is not feasible.
- Only use an extension method if the functionality being added is a ‘core’
  general feature that would be appropriate to add to the source of the
  original class.
- Note - if we have the source to the class being extended, and the
  maintainer of the original class does not want to add the function,
  prefer not using an extension method.
- Only put extension methods into core libraries that are available
  everywhere - extensions that are only available in some code will become a
  readability issue.
- Be aware that using extension methods always obfuscates the code, so err on
  the side of not adding them.

### ref and out

- Use `out` for returns that are not also inputs.
- Place `out` parameters after all other parameters in the method definition.
- `ref` should be used rarely, when mutating an input is necessary.
- Consider using `var` and `out` together.

### LINQ

- In general, prefer single line LINQ calls and imperative code, rather than
  long chains of LINQ. Mixing imperative code and heavily chained LINQ is
  often hard to read.
- Prefer member extension methods over SQL-style LINQ keywords - e.g. prefer
  `myList.Where(x)` to `myList where x`.
- Avoid `Container.ForEach(...)` for anything longer than a single statement.
- Avoid using it in Unity's MonoBehaviour `Update()` and `FixedUpdate()`; use it only in one-off calls such as `Awake()` and `Start()`.

### Array vs List
- Array and List, the use of array is preferred.
- Generally, arrays perform better than `List<>`.
- After selecting from `List<>` using Linq, it can be converted to an array using `ToArray()`.

### Namespace
TODO

### String

- String uses an empty string instead of null. `var someString = String.Empty;`
- String interpolation vs `String.Format()` vs `String.Concat` vs `operator+`
    - In general, use whatever is easiest to read, particularly for logging and
  assert messages.
    - Be aware that chained `operator+` concatenations will be slower and cause
  significant memory churn.
    - If performance is a concern, `StringBuilder` will be faster for multiple string concatenations.

### Constructor vs Object Initializer syntax
```c#
// Object initializer syntax
var x = new SomeClass 
{
    Property1 = value1,
    Property2 = value2,
};

// Constructor
var x = new SomeClass(value1, value2)
```

- Avoid using object initializer syntax for classes or structs with constructors.
- If splitting across multiple lines, indent one block level.

### Calling delegates

- When calling a delegate, use `Invoke()` and use the null conditional
  operator - e.g. `SomeDelegate?.Invoke()`. This clearly marks the call at the
  callsite as ‘a delegate that is being called’. The null check is concise and
  robust against threading race conditions.

### The `var` keyword

- Use of `var` is encouraged if it aids readability by avoiding type names
  that are noisy, obvious, or unimportant.
- Encouraged:
    - When the type is obvious - e.g. `var apple = new Apple();`, or `var
      request = Factory.Create<HttpRequest>();`
    - For transient variables that are only passed directly to other methods -
      e.g. `var item = GetItem(); ProcessItem(item);`
- Discouraged:
    - When working with basic types - e.g. `var success = true;`
    - When working with compiler-resolved built-in numeric types - e.g. `var
      number = 12 * ReturnsFloat();`
    - When users would clearly benefit from knowing the type - e.g. `var
      listOfItems = GetList();`

### Region

## (TODO) Unity coding guidelines

### Unity callbacks

### Refer to the interface in Inspector

### SerializeField

### Destroy

### Editor extension

### Symbols

### Dependency injection

### Tag & Layer

### Open two inspector

### Initialize

### Update & FixedUpdate

### Change variable name
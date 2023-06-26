# Contributing

## Contributing to Andino

The following is a set of guidelines for contributing to Andino project.
These are mostly guidelines, not rules. Use your best judgment, and feel free to
propose changes to this document in a pull request.


## Code of Conduct

This project and everyone participating in it is governed by the [CODE OF CONDUCT](CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code.


## How to Contribute


### Reporting Bugs

#### Before Submitting a Bug Report

1. Determine the repository which should receive the problem.
2. Search the repository's issues to see if the same or similar problem has
   been opened. If it has and the issue is still open, then add a comment to
   the existing issue. Otherwise, create a new issue.

#### How to Submit a Good Bug Report

Create an issue on the repository that is related to your bug, explain the
problem, and include additional details to help maintainers reproduce the
problem. Refer to the `Short, Self Contained, Correct (Compilable), Example
[Guide](http://sscce.org/) as well as the following tips:

* **Use a clear and descriptive title** for the issue to identify the problem.
* **Describe the exact steps which reproduce the problem** in as many details as possible. When listing steps, don't just say what you did, but explain how you did it.
* **Provide specific examples to demonstrate the steps.** Include links to files or projects, or copy/pasteable snippets, which you use in those examples.
* **Describe the behavior you observed after following the steps** and point out what exactly is the problem with that behavior.
* **Explain which behavior you expected to see instead and why**.
* **Include screenshots and animated GIFs** which show you following the described steps and clearly demonstrate the problem.
* **If the problem wasn't triggered by a specific action**, describe what you were doing before the problem happened and share more information using the guidelines below.

Provide more context by answering these questions:

* **Did the problem start happening recently** (e.g. after updating to a new version) or was this always a problem?
* If the problem started happening recently, **can you reproduce the problem in an older version?** What's the most recent version in which the problem doesn't happen?
* **Can you reliably reproduce the issue?** If not, provide details about how often the problem happens and under which conditions it normally happens.

Include details about your configuration and environment:

* **Which version of Andino are you using?**?
* **What's the name and version of the OS you're using**?
* **Are you running Andino using the provided docker container?** See [docker](docker/README.md).
* **Are you running Andino in a virtual machine?** If so, which VM software are you using and which operating systems and versions are used for the host and the guest?


### Suggesting Enhancements

This section guides you through submitting an enhancement suggestion,
including completely new features and minor improvements to existing
functionality. Following these guidelines helps maintainers and the
community understand your suggestion and find related suggestions.

Before creating enhancement suggestions, please check [`before-submitting-a-bug-report`](#before-submitting-a-bug-report) as you
might find out that you don't need to create one. When you are creating an
enhancement suggestion, please include as many details as possible.
When filling in the issue form for an enhancement suggestion, include the
steps that you imagine you would take if the feature you're requesting
existed.

#### Before Submitting An Enhancement Suggestion

* **Check if you're using the latest software version**. A more recent version may contain your desired feature.
* **Determine which repository the enhancement should be suggested in**
* **Perform a** [cursory search](https://github.com/Ekumen-OS/andino/issues?q=is%3Aissue) to see if the enhancement has already been suggested. If it has, add a comment to the existing issue instead of opening a new one.

#### How Do I Submit A (Good) Enhancement Suggestion

Enhancement suggestions are tracked as [GitHub
issues](https://help.github.com/en/github/managing-your-work-on-github/about-issues).
After you've determined which repository your enhancement suggestion is related to, create an issue on that repository and provide the following information:

* **Use a clear and descriptive title** for the issue to identify the suggestion.
* **Provide a step-by-step description of the suggested enhancement** in as many details as possible.
* **Provide specific examples to demonstrate the steps**. Include copy/pasteable snippets which you use in those examples, as [Markdown code blocks](https://help.github.com/en/github/writing-on-github/creating-and-highlighting-code-blocks).
* **Describe the current behavior** and **explain which behavior you expected to see instead** and why.
* **Include screenshots and animated GIFs** which show you following the described steps and clearly demonstrate the problem.
* **Explain why this enhancement would be useful** to most users and isn't something that can or should be implemented as a separate application.
* **Specify which version of Andino you're using.**
* **Specify the name and version of the OS you're using.**

### Contributing Code

We follow a development process designed to reduce errors, encourage
collaboration, and make high quality code. Review the following to
get acquainted with this development process.

#. **Read the** [reporting_bugs](#reporting-bugs) **and** [suggesting_enhancements](#suggesting-enhancements) **sections first.**

#. **Fork the Andino package** you want to contribute to. This will create
   your own personal copy of the package. All of your development should
   take place in your fork.
   - An important thing to do is create a remote pointing to the [upstream remote repository](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/configuring-a-remote-for-a-fork). This way, you can always check for modifications on the original repository and **always** keep your fork repository up to date.

#. **Work out of a new branch**, one that is not
   a release / main branch. This is a good habit to get in, and will make
   your life easier.

#. **Write your code.** To remember:
   - Look at the existing code and try to maintain the existing style and pattern as much as possible
   - **Always** keep your branch updated with the original repository

#### Process

All Andino team members actively:

* **Watch** all Andino-related repositories to receive email notifications of new issues / pull requests
* Provide **feedback** to issues as soon as possible
* **Review** pull requests as soon as possible

  * Team members can review pull requests already under review or approved
  * Team members can provide some feedback without doing a full review

Pull requests can be merged when:

* They have at least 1 approval from a member of the core team
* There are no unresolved comments
* CI is passing
* [Developer Certificate of Origin](https://developercertificate.org/)(DCO) check is passing

  * DCO is a declaration of ownership, basically saying that you created the contribution and that it is suitable to be covered under an open source license (not proprietary).
  * All you have to do is end your commit message with Signed-off-by: Your Full Name <your.name@email.com>
  * If your user.name and user.email configurations are set up in git, then you can simply run git commit -s to have your signature automatically appended.

Merging strategy:

* For internal contributions, give the original author some time to hit the merge button themselves / check directly with them if it’s ok to merge.
* Default to “squash and merge”
  * Review the pull request title and reword if necessary since this will be part of the commit message.
  * Make sure the commit message concisely captures the core ideas of the pull request and contains all authors' signatures.

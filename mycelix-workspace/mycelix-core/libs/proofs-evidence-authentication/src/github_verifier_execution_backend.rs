use crate::{
    ExecutedGitHubVerificationV1, GitObjectIdV1, GitHubPublicVerifierExecutionPolicyV1,
    GitHubVerifierExecutionPolicyErrorV1, GitHubVerifierExecutorErrorV1,
    QualificationReceiptV1, ReceiptAuthenticationPolicyV1,
    build_github_public_command_plan_with_execution_policy_v1,
};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GitHubBoundVerifierExecutionErrorV1 {
    CommandPlan(GitHubVerifierExecutionPolicyErrorV1),
    Executor(GitHubVerifierExecutorErrorV1),
}

/// Execute the GitHub public-verifier profile only after rebuilding its sealed command
/// plan from the exact execution policy, authentication policy, receipt and admitted
/// signer revision supplied to this call.
///
/// The command plan is deliberately not accepted from the caller. This prevents a
/// structurally valid plan created under policy A from being replayed beside policy B.
pub fn execute_github_public_verifier_v1(
    execution_policy: &GitHubPublicVerifierExecutionPolicyV1,
    authentication_policy: &ReceiptAuthenticationPolicyV1,
    receipt: &QualificationReceiptV1,
    selected_signer_revision: GitObjectIdV1,
    working_directory: &str,
) -> Result<ExecutedGitHubVerificationV1, GitHubBoundVerifierExecutionErrorV1> {
    let command_plan = build_github_public_command_plan_with_execution_policy_v1(
        execution_policy,
        authentication_policy,
        receipt,
        execution_policy.expected_verifier(),
        selected_signer_revision,
        working_directory,
    )
    .map_err(GitHubBoundVerifierExecutionErrorV1::CommandPlan)?;

    crate::github_verifier_executor::execute_github_public_verifier_v1(
        execution_policy,
        &command_plan,
        receipt,
    )
    .map_err(GitHubBoundVerifierExecutionErrorV1::Executor)
}
